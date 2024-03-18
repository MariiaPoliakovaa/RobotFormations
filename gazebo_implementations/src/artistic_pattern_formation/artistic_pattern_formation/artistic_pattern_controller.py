import rclpy
import ipdb
import rvo2
import yaml
import json
import time
import os
import logging
import numpy as np

from copy import deepcopy
from pathlib import Path
from shapely import Polygon
from itertools import cycle
from scipy.optimize import linear_sum_assignment

from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist

from typing import List, Dict

from turtlebot_controller.turtlebot_controller import Turtlebot, Point

from .artistic_pattern_gen import create_rectangle, create_circle, create_triangle, goal_generation, euclidean_dist


class ArtisticPatternController(Node):
    def __init__(self, robot_names: List[str], patterns: Dict[str, Polygon]):
        """
        :param robot_names : A list of robot names
        :param patterns    : A dict with the names and polygons of patterns
        """
        super().__init__("artistic_pattern_controller")
        self.max_angular_speed = 0.6
        self.max_linear_speed = 0.6 # TODO: get turtlebot max speed
        self.targets_set = False
        self.seconds = 0
        self.last_seconds = 0

        # make an endless cycling iterator for the patterns
        self.pattern_names = cycle(patterns.keys())
        self.patterns = patterns

        # init turtlebot objects
        self.robot_names = robot_names
        self.robots = {
            name: Turtlebot(name,
                            self.max_angular_speed,
                            self.max_linear_speed
            ) for name in robot_names}

        # init targets for robots
        self.robot_goals = {name: None for name in robot_names}

        # init subscriptions and publishers
        self.odom_subscriptions = self.init_subscriptions(robot_names)
        self.twist_publishers = self.init_twist_publishers(robot_names)

    def update_step(self):
        """
        If all robot positions were updated at least once.

        Update all agent positions in the simulation, do one step
        and update all robots speed based on the simulation
        """
        # chech if all robots had their position updated
        update = True
        for robot in self.robots.values():
            if robot.updated is not True:
                update = False

        if not update:
            return

        # set update flag to false
        for robot in self.robots.values():
            robot.updated = False

        # update robot positions and preferred velocities
        for name in self.robot_names:
            agent = self.sim_agents[name]
            robot = self.robots[name]

            # update robot position
            position = (robot.position.x, robot.position.y)
            self.sim.setAgentPosition(agent, position)

            # update simulation preferred velocity
            _, preferred_velocity = self.preferred_velocity(robot)
            self.sim.setAgentPrefVelocity(agent, preferred_velocity)

            # update simulation robot velocity
            self.sim.setAgentVelocity(agent, robot.velocity)

        # set the time step of the simulation
        time_delta = self.seconds - self.last_seconds
        self.sim.setTimeStep(time_delta)

        # do one simulation step
        self.sim.doStep()

        # update speed for each robot
        for name in self.robot_names:
            agent = self.sim_agents[name]
            robot = self.robots[name]

            velocity = self.sim.getAgentVelocity(agent)
            speed = np.sqrt(velocity[0]**2 + velocity[1]**2)

            robot.linear_speed = speed


    def init_simulation(self):
        """
        Initializes the RVO simulation. Must be executed after
        each robot has it's initial position
        """
        self.get_logger().debug("Initializing simulation")

        # Define variables for simulation
        time_step = 1/60. # really irrelevant
        max_neighbor_dist = 0.5
        max_neighbors = 5 # must be higher with many close robots, but takes longer
        time_horizon = 0.5
        time_horizon_obst = 2
        agent_radius = 0.5 # TODO: get turtlebot radius
        max_speed = self.max_linear_speed

        self.sim = rvo2.PyRVOSimulator(
            time_step,
            max_neighbor_dist,
            max_neighbors,
            time_horizon,
            time_horizon_obst,
            agent_radius,
            max_speed
        )

        # init empyt dict for simulation robots
        self.sim_agents = {}

        # add the agents
        for robot_name, robot in self.robots.items():
            # add robot to simulation
            agent = self.sim.addAgent((robot.position.x, robot.position.y))

            # set preferred velocity for robot
            _, preferred_velocity = self.preferred_velocity(robot)
            self.sim.setAgentPrefVelocity(agent, preferred_velocity)

            # save agent
            self.sim_agents[robot_name] = agent

    def preferred_velocity(self, robot):
        """
        Computes the preferred speed for the provided robot

        In the paper the velocity is computed. That means
        it is a vector. As we rotate the robot before moving,
        the speed should suffice.
        The second term is normalized and represents the direction
        vector.

        Problem: The robots speed up very quickly even if their
        target position is very close to them. This can cause them
        to overshoot their target or be unable to react in time to
        other robots. I propose reducing the robots speed if it is
        close to it's target.
        """
        Vp = 0.8 * self.max_linear_speed

        position = robot.position
        initial_position = robot.initial_position
        goal = self.robot_goals[robot.name]

        # Make vectors for position and goal
        position_v = np.array([position.x, position.y])
        goal_v = np.array([goal.x, goal.y])

        # calculate K
        K = initial_position.distance_to(goal)

        # calculate preferred speed
        preferred_speed = Vp * np.min(np.array([
            1,
            position.distance_to(goal) / K
        ]))

        # reduce speed if starting position is close to it's target
        min_dist = 1
        if initial_position.distance_to(goal) < min_dist:
            reduction_factor = np.min(np.array([
                1,
                (min_dist - initial_position.distance_to(goal)) / min_dist
            ]))

            preferred_speed *= reduction_factor

        # clip minimum speed to 0.1 (Not originally part of the algorithm)
        preferred_speed = preferred_speed if preferred_speed > 0.1 else 0.1

        # calculate preferred velocity
        preferred_velocity = preferred_speed * (goal_v - position_v) / position.distance_to(goal)
        preferred_velocity = (preferred_velocity[0], preferred_velocity[1])

        return preferred_speed, preferred_velocity

    def assign_targets(self):
        """
        Generates target positions from the pattern and assigns
        those to the robots

        :param pattern : The pattern used to generate target positions
        """
        t1 = time.time()
        # use next pattern
        pattern_name = next(self.pattern_names)
        pattern = self.patterns[pattern_name]

        self.get_logger().info("Computing goal positions for {}".format(pattern_name))
        goal_positions = goal_generation(pattern, len(self.robots))

        # create distance matrix
        n = len(self.robot_names)
        distances = np.zeros((n, n))
        robot_names = []
        for i, (robot_name, robot) in enumerate(self.robots.items()):
            robot_names.append(robot_name)
            for j, goal_position in enumerate(goal_positions):
                dist = np.sqrt( (robot.position.x - goal_position[0])**2 +
                               (robot.position.y - goal_position[1])**2 )
                distances[i][j] = dist**2

        # do target assignement
        assigned_positions = linear_sum_assignment(distances, maximize=False)

        for robot_idx, goal_idx in zip(assigned_positions[0], assigned_positions[1]):
            robot_name = robot_names[robot_idx]
            robot = self.robots[robot_name]
            position = robot.position
            goal_position = goal_positions[goal_idx]
            self.robot_goals[robot_name] = Point(
                goal_position[0],
                goal_position[1]
            )
            robot.set_target(
                self.robot_goals[robot_name]
            )

        # calculate the elapsed time
        t2 = time.time()
        self.get_logger().info("Target assignement took {:.2f}s".format(t2-t1))

    def all_positions_assigned(self):
        """
        Returns True if all turtlebots know their
        positions, else False
        """
        positions_set = True
        for robot in self.robots.values():
            if robot.position is None or robot.initial_position is None:
                positions_set = False
        return positions_set

    def pattern_achieved(self) -> bool:
        """
        Returns True if each robot has reached its goal position,
        otherwise False
        """
        done = True
        for robot_name, robot in self.robots.items():
            robot_goal = self.robot_goals[robot_name]
            if not robot.reached_target():
                done = False
        return done

    def targets_assigned(self):
        """
        Returns True if all robots have their target assigned,
        else False
        """
        assigned = True
        for robot in self.robots.values():
            if robot.target is None:
                assigned = False
        return assigned


    def init_twist_publishers(self, robot_names: List[str]) -> Dict[str, Publisher]:
        """
        Initializes the Twist message publisher for each
        turtlebot.

        :param robot_names : List of names of the turtlebots
        :returns           : Dictionary with the robot names as keys and the
                             publishers as values
        """
        publishers = {}
        for robot_name in robot_names:
            topic_name = f'/{robot_name}/cmd_vel'
            publisher = self.create_publisher(
                Twist,
                topic_name,
                10
            )
            publishers[robot_name] = publisher
        return publishers

    def init_subscriptions(self, robot_names: List[str]) -> Dict[str, Subscription]:
        """
        Initializes the subscriptions to the odometry message
        for each turtlebot.

        :param robot_names: List of names of the turtlebots
        :returns           : Dictionary with the robot names as keys and the
                             subscriptions as values
        """
        subscriptions = []

        # subscribe to odometry messages for each robot
        for robot_name in self.robot_names:
            topic_name = f'/{robot_name}/odom'
            subscription = self.create_subscription(
                Odometry,
                topic_name,
                lambda msg, robot_name=robot_name: self.odom_callback(msg, robot_name),
                10
            )

        # I need to specifically set the profile, wont work otherwise
        clock_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_ALL,
            durability=DurabilityPolicy.VOLATILE,
        )

        # subscribe to time message from gazebo
        time_subscription = self.create_subscription(
            Clock,
            '/clock',
            self.timer_callback,
            clock_qos_profile
        )
        return subscriptions

    def timer_callback(self, time_msg):
        """
        Updates the time based on /clock messages from gazebo
        """
        self.last_seconds = self.seconds
        self.seconds = time_msg.clock.sec + time_msg.clock.nanosec / 1e9

    def reset_robots(self):
        """
        Unsets the targets for all robots
        """
        for robot in self.robots.values():
            robot.target = None
            robot.stop_moving()
            robot.initial_position = None
        self.targets_set = False

    def robots_correctly_oriented(self):
        """
        Returns true if all robots are correctly oriented
        """
        correctly_oriented = True

        for robot in self.robots.values():
            if not robot.correctly_oriented:
                correctly_oriented = False

        return correctly_oriented

    def odom_callback(self, odom_msg: Odometry, robot_name: str):
        """
        Odometry callback to receive pose from the turtlebot and
        send a twist message based on the turtlebots state

        The collision avoidance algorithm is implemented in RVO2:
            https://gamma.cs.unc.edu/RVO2/documentation/2.0/index.html.
        I use the following repository:
            https://github.com/sybrenstuvel/Python-RVO2.git
        It provides python bindings for the RVO2 library.

        The idea is to initialize the RVO2 simulation once the start and goal
        positions for each robot are computed.
        Then I can update the simulation step every time all robots had their
        callback function executed and update the position in the simulation
        for each robot and update the speed of each robot based on the
        simulation.

        1. Get position and goal for each robot
        2. Start ROV2 simulation with positions and preferred velocities
        3. call simulation.step() once each robot has updated its position once
        4. Update robot positions in ROV2 simulation based on actual position
        5. Update actual robot speed based on robot velocity in ROV2 simulation
        6. repeat steps 3-5 until all robots reached their target
        """

        self.get_logger().debug("===========================")
        self.get_logger().debug("Received odom message for {}".format(robot_name))

        # if positions are not yet assigned and all robot positions were received
        if not self.targets_set and self.all_positions_assigned():
            # assign targets to robots
            self.assign_targets()

            # start the RVO2 simulation
            self.init_simulation()
            self.targets_set = True

        # get the robot
        robot = self.robots[robot_name]

        # get the publisher corresponding to robot
        twist_publisher = self.twist_publishers[robot_name]

        # read robot positions
        robot.position = Point(
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y
        )

        # read robot velocity
        robot.velocity = (
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y
        )

        # signify that the robot had its state updated
        robot.updated = True

        # Save initial position
        if robot.initial_position is None:
            robot.initial_position = robot.position

        # Initialize twist message
        twist_msg = Twist()

        # if the robots target is set and its not reached its target
        if robot.target is not None and not robot.reached_target() and self.all_positions_assigned():

            # RVO2 Simulation step
            self.update_step()

            # rotate it towards its target
            twist_msg = robot.rotate_to_target(odom_msg, twist_msg)

            # if it aim towards its target start moving
            if self.robots_correctly_oriented():
                robot.start_moving()

            # if the robot is supposed to run and has not reached its target
            if robot.run == True:
                twist_msg = robot.set_linear_speed(twist_msg)

        # publish twist message
        twist_publisher.publish(twist_msg)

        # if pattern is achieved, start next pattern
        if self.targets_assigned():
            if self.pattern_achieved():
                self.get_logger().info("Next pattern")

                # unset previous targets
                self.reset_robots()

def create_patterns():
    circle_center = (0, 0)
    circle_radius = 1.0
    circle_polygon = create_circle(circle_center, circle_radius)

    # Create a triangle
    triangle_vertices = [(-1.0, -1.0), (1.0, -1.0), (0.0, 1.0)]
    triangle_polygon = create_triangle(triangle_vertices)

    # Create a rectangle
    rectangle_lower_left = (-1.0, -1.0)
    rectangle_upper_right = (1.0, 1.0)
    rectangle_polygon = create_rectangle(rectangle_lower_left, rectangle_upper_right)

    patterns = {
        'circle': circle_polygon,
        'triangle': triangle_polygon,
        'rectangle': rectangle_polygon
    }

    return patterns


def main():
    rclpy.init()

    # get robots
    robots = json.loads(os.environ.get("ROBOTS"))
    robots = [robot['name'] for robot in robots]

    # target patterns
    patterns = create_patterns()

    # Initialize target assignement controller
    tac = ArtisticPatternController(robots, patterns)

    rclpy.spin(tac)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
