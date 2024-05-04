import rclpy
import csv
import threading
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
from datetime import datetime

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist

from typing import List, Dict

from turtlebot_controller.turtlebot_controller import Point

from .artistic_pattern_gen import create_rectangle, create_circle,create_triangle, goal_generation, euclidean_dist, create_star
from .turtlebot import ArtisticTurtleBot as Turtlebot

BASE_DIR = Path(__file__).resolve().parents[6]

def append_to_csv(filename, data):
    file_exists = os.path.exists(filename)

    if not file_exists:
        # create file
        with open(filename, 'w') as f:
            pass

    with open(filename, 'a', newline='') as file:
        writer = csv.writer(file)
        if not file_exists:
            # If the file doesn't exist, write the header
            writer.writerow(['Pattern', 'Time'])  # Example header
        writer.writerow(data)


class ArtisticPatternController(Node):
    def __init__(self, robot_names: List[str], goal_positions: Dict[str, Polygon]):
        """
        :param robot_names : A list of robot names
        :param patterns    : A dict with the names and polygons of patterns
        """
        super().__init__("artistic_pattern_controller")
        self.max_angular_speed = 2.84
        self.max_linear_speed = 0.22 # TODO: get turtlebot max speed
        self.max_delta_velocity = 0.6
        self.max_delta_omega = np.pi/2
        self.targets_set = False
        self.seconds = 0
        self.last_seconds = 0
        self.sim = None
        self.turtlebot_radius = 0.07

        # set up timing logfile
        current_timestamp = datetime.now().strftime("%Y_%m_%d:%H_%M_%S")
        timing_file = 'artistic_pattern_gen_{}.csv'.format(current_timestamp)
        self.timing_path = Path(BASE_DIR, 'timing', timing_file)

        # make an endless cycling iterator for the patterns
        self.pattern_names = cycle(goal_positions.keys())
        self.goal_positions = goal_positions

        # init turtlebot objects
        self.robot_names = robot_names
        self.robots = {
            name: Turtlebot(name,
                            self.max_angular_speed,
                            self.max_linear_speed,
                            self.max_delta_velocity,
                            self.max_delta_omega
            ) for name in robot_names}

        # init targets for robots
        self.robot_goals = {name: None for name in robot_names}

        # init subscriptions and publishers
        self.odom_subscriptions = self.init_subscriptions(robot_names)
        self.twist_publishers = self.init_twist_publishers(robot_names)

        # initialize start timer to start main loop after two seconds
        #self.start_timer = self.create_timer(2, self.start_timer_callback)

    def start_timer_callback(self):
        """
        This callback gets called executed by a timer shortly after
        the node was initialized. This is needed because we need
        to wait a moment for all robots to have published their
        location.
        This function starts the main loop of the Node
        """
        # cancel the timer to only execute once
        self.start_timer.cancel()

        # run main loop
        self.run()



    def init_simulation(self, update_rate: int):
        """
        Initializes the RVO simulation. Must be executed after
        each robot has it's initial position
        """
        self.get_logger().info("Initializing simulation")

        # Define variables for simulation
        max_neighbor_dist = 2 * self.turtlebot_radius
        # NOTE: We can probably equal max_neighbors to the number of robots, 
        # the RVO2 simulation should not be the bottleneck
        time_step = 1/update_rate
        max_neighbors = len(self.robot_names) - 1 # must be higher with many close robots, but takes longer
        time_horizon = 3
        time_horizon_obst = 2 # doesnt matter, we have no obstacles
        agent_radius = self.turtlebot_radius
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
            goal = self.robot_goals[robot_name]
            preferred_velocity = robot.preferred_velocity(goal)
            self.sim.setAgentPrefVelocity(agent, preferred_velocity)

            # save agent
            self.sim_agents[robot_name] = agent

    def assign_targets(self):
        """
        Generates target positions from the pattern and assigns
        those to the robots

        :param pattern : The pattern used to generate target positions
        """
        # use next pattern
        pattern_name = next(self.pattern_names)
        self.pattern_name = pattern_name
        goal_positions = self.goal_positions[pattern_name]
        self.get_logger().info("Next pattern: {}".format(pattern_name))

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

    def log_timing(self):
        """
        Writes the current timing of the current
        formation to csv file
        """
        data = [self.pattern_name, "{:.2f}".format(self.seconds)]
        append_to_csv(self.timing_path, data)

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
            robot.correctly_oriented = False
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

    def update_step(self):
        """
        Update all agent positions in the simulation, do one step
        and update all robots speed based on the simulation
        """
        # update robot positions and preferred velocities
        for name in self.robot_names:
            agent = self.sim_agents[name]
            robot = self.robots[name]

            # update robot position
            position = (robot.position.x, robot.position.y)
            self.sim.setAgentPosition(agent, position)

            # update simulation preferred velocity
            goal = self.robot_goals[name]
            preferred_velocity = robot.preferred_velocity(goal)
            #self.get_logger().info(
            #    "Preferred velocity: {}".format(str(preferred_velocity))
            #)

            # set preferred velocity
            self.sim.setAgentPrefVelocity(agent, preferred_velocity)

            # update simulation robot velocity
            self.sim.setAgentVelocity(agent, robot.velocity)

        # do one simulation step
        self.sim.doStep()

    def send_controll_signals(self, time_step: float):
        """
        For each robot: Sends the control signal to
        the robot

        :param time_step : Length of a simulation time step
        """
        for robot in self.robots.values():
            # get the publisher corresponding to robot
            twist_publisher = self.twist_publishers[robot.name]

            # Initialize twist message
            twist_msg = Twist()

            # start moving if all robots are correctly oriented
            if self.robots_correctly_oriented():
                #self.get_logger().info("Robots correctly oriented")
                robot.start_moving()

            if not robot.run:
                # rotate robot towards its target
                twist_msg = robot.rotate_to_target(twist_msg)
            elif robot.reached_target():
                #self.get_logger().info("Stop moving")
                robot.stop_moving()
            else:
                # get simulation agent
                agent = self.sim_agents[robot.name]

                # get target velocity of agent
                velocity = self.sim.getAgentVelocity(agent)
                #self.get_logger().info("Agent velocity: {}".format(str(velocity)))

                # calculate target rotation and speed
                target_speed = np.sqrt(velocity[0]**2 + velocity[1]**2)
                target_rotation = np.arctan2(velocity[1], velocity[0]) % (2*np.pi)
                target_rotation -= np.pi
                robot_rotation = robot.w - np.pi

                rotation_delta = target_rotation - robot_rotation

                angular_vel = robot.angular_speed * rotation_delta/(2*np.pi)

                #self.get_logger().info("------------")
                #self.get_logger().info("Robot position: ({:.2f}, {:.2f})".format(robot.position.x,robot.position.y))
                #self.get_logger().info("Robot target: ({:.2f}, {:.2f})".format(robot.target.x,robot.target.y))
                #self.get_logger().info("Robot rotation: {:.2f}".format(robot_rotation))
                #self.get_logger().info("Target rotation: {:.2f}".format(target_rotation))
                #self.get_logger().info("Rotation delta: {:.2f}".format(rotation_delta))

                # set values
                twist_msg.linear.x = target_speed
                twist_msg.angular.z = angular_vel


            # publish twist message
            twist_publisher.publish(twist_msg)


    def odom_callback(self, odom_msg: Odometry, robot_name: str):
        """
        Odometry callback to receive pose from the turtlebot and
        send a twist message based on the turtlebots state

        The collision avoidance algorithm is implemented in RVO2:
            https://gamma.cs.unc.edu/RVO2/documentation/2.0/index.html.
        I use the following repository:
            https://github.com/sybrenstuvel/Python-RVO2.git
        It provides python bindings for the RVO2 library.
        4. Update robot positions in ROV2 simulation based on actual position
        5. Update actual robot speed based on robot velocity in ROV2 simulation
        6. repeat steps 3-5 until all robots reached their target
        """
        robot = self.robots[robot_name]

        # get x and y pos
        robot.position = Point(
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y
        )

        # get rotation
        robot.w = robot.get_orientation(odom_msg)

        # get x and y velocity
        robot.velocity = (
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y
        )

        # Save initial position
        if robot.initial_position is None:
            robot.initial_position = robot.position


    def run(self):
        # define rate at which to send messages
        update_rate = 5 # /s
        real_time_factor = 0.3
        sim_update_rate = update_rate * real_time_factor

        # start rate
        self.rate = self.create_rate(sim_update_rate)

        while rclpy.ok():
            if self.all_positions_assigned():
                t1 = time.time()

                if not self.targets_assigned():
                    self.assign_targets()
                else:
                    if self.sim is None:
                        # Init RVO simulation
                        self.init_simulation(update_rate)

                    # synch robot states and do simulation step
                    self.update_step()

                    # send controll signals as in simulation
                    self.send_controll_signals(1/update_rate)

                if self.pattern_achieved():
                    self.reset_robots()

                # warn if timing constraint violated
                t2 = time.time()
                td = t2 - t1
                if td > 1/update_rate:
                    self.get_logger().warning("td > update_rate: {:.2f} > 1/{}".format(td, update_rate))

            self.rate.sleep()

def main():
    rclpy.init()

    # get robots
    robots = json.loads(os.environ.get("ROBOTS"))
    robots = [robot['name'] for robot in robots]

    # target patterns
    goal_positions = os.environ.get("TARGET_POSITIONS")

    if goal_positions is not None:
        goal_positions = json.loads(goal_positions)
    else:
        raise ValueError("No target patterns in environment")

    # Initialize target assignement controller
    controller = ArtisticPatternController(robots, goal_positions)

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(controller, ), daemon=True)
    thread.start()

    controller.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
