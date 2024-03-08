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
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist

from typing import List, Dict

from .turtlebot import Turtlebot, Point
from .artistic_pattern_gen import create_rectangle, create_circle, create_triangle, goal_generation, euclidean_dist


class ArtisticPatternController(Node):
    def __init__(self, robot_names: List[str], patterns: List[Polygon]):
        """
        :param robot_names : A list of robot names
        :param patterns    : The patterns that are used for the formations
        """
        super().__init__("artistic_pattern_controller")
        self.max_angular_speed = 0.5
        self.max_linear_speed = 0.5 # TODO: get turtlebot max speed
        self.targets_set = False
        self.last_seconds = 0

        # make an endless cycling iterator for the patterns
        self.patterns = cycle(patterns)

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

            # update preferred velocity
            preferred_velocity = self.preferred_velocity(robot)
            self.sim.setAgentPrefVelocity(agent, preferred_velocity)

            # update actual robot velocity
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
        """
        Vp = 0.8 * self.max_linear_speed

        position = robot.position
        initial_position = robot.initial_position
        goal = self.robot_goals[robot.name]

        # Make vectors for position and goal
        try:
            position_v = np.array([position.x, position.y])
        except:
            ipdb.set_trace()
        goal_v = np.array([goal.x, goal.y])

        # calculate K
        K = initial_position.distance_to(goal)

        # calculate preferred speed
        preferred_speed = Vp * np.min(np.array([
            1,
            position.distance_to(goal) / K
        ]))

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
        # use next pattern
        pattern = next(self.patterns)

        # goal positions equal pattern, as
        # we pre-compute patterns to save on compute time
        goal_positions = pattern

        # create distance matrix
        n = len(self.robot_names)
        distances = np.zeros((n, n))
        robot_names = []
        self.get_logger().info("Creating distance matrix")
        for i, (robot_name, robot) in enumerate(self.robots.items()):
            robot_names.append(robot_name)
            for j, goal_position in enumerate(goal_positions):
                dist = np.sqrt( (robot.position.x - goal_position[0])**2 +
                               (robot.position.y - goal_position[1])**2 )
                distances[i][j] = dist**2

        # do target assignement
        self.get_logger().info("Doing target assignement")
        assigned_positions = linear_sum_assignment(distances, maximize=False)
        for robot_idx, goal_idx in zip(assigned_positions[0], assigned_positions[1]):
            robot_name = robot_names[robot_idx]
            goal_position = goal_positions[goal_idx]
            self.robot_goals[robot_name] = Point(
                goal_position[0],
                goal_position[1]
            )

    def all_positions_assigned(self):
        """
        Returns True if all turtlebots know their
        positions, else False
        """
        positions_set = True
        for robot in self.robots.values():
            if robot.position is None:
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
            if robot.position.distance_to(robot_goal) > 0.01:
                done = False
        return done


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
            subscriptions.append(subscription)

        # subscribe to time message from gazebo
        time_subscription = self.create_subscription(
            Clock,
            '/clock',
            self.timer_callback,
            10
        )
        return subscriptions

    def timer_callback(self, time_msg):
        """
        Updates the time based on /clock messages from gazebo
        """
        self.last_seconds = self.seconds
        self.seconds = time_msg.clock.sec + time_msg.clock.nanosec / 1e9

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

        self.get_logger().info("===========================")
        self.get_logger().info("Received odom message for {}".format(robot_name))

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
        if robot.target is not None and not robot.reached_target():

            # rotate it towards its target
            self.get_logger().info("Rotate to target")
            twist_msg, correctly_oriented = robot.rotate_to_target(odom_msg, twist_msg)

            # if it aim towards its target start moving
            if correctly_oriented:
                self.get_logger().info("Enable moving")
                robot.start_moving()

            # if the robot is supposed to run and has not reached its target
            if robot.run == True:
                self.get_logger().info("Start moving")
                twist_msg = robot.set_linear_speed(twist_msg)

        # publish twist message
        twist_publisher.publish(twist_msg)

        # if pattern is achieved, start next pattern
        if self.all_positions_assigned():
            try:
                if self.pattern_achieved():
                    self.get_logger().info("Next pattern")
                    self.targets_set = False
            except:
                for robot in self.robots.values():
                    self.get_logger().info("{} position: {}".format(robot.name, robot.position))


def main():
    rclpy.init()

    # robot names
    robots = ['Tony', 'Jacky', 'Andreas']#, 'Susan', 'Pikachu', 'Brudi']
              #'DarthVader', 'Lisa', 'Gertrude']

    #ipdb.set_trace()
    data_dir = Path(__file__).resolve().parent.parent.parent.parent.parent / 'share/data/artistic_pattern_formation'
    with open(Path(data_dir, 'patterns.json'), 'r') as f:
        goal_positions = json.load(f)

    # goal positions to numpy arrays
    goal_positions = [np.array(goal_pos) for goal_pos in goal_positions]

    # Initialize target assignement controller
    tac = ArtisticPatternController(robots, goal_positions)

    rclpy.spin(tac)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
