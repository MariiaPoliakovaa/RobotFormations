import rclpy
import csv
import threading
import ipdb
import yaml
import json
import time
import os
import logging
import numpy as np
import pandas as pd

from copy import deepcopy
from pathlib import Path
from shapely import Polygon
from itertools import cycle
from scipy.optimize import linear_sum_assignment
from datetime import datetime
from typing import List, Dict

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist

BASE_DIR = Path(__file__).resolve().parents[6]

class BaseController(Node):
    def __init__(self, node_name, robot_names, patterns):
        super().__init__(node_name)

        # Init a dictionary to log robot positions
        self.positions = {
            name: [] for name in robot_names
        }
        self.positions['timestamp'] = []

        # Simulation time
        self.simulation_time = None

        # Turtlebot parameters
        self.max_angular_speed = 2.84
        self.max_linear_speed = 0.22
        self.max_delta_velocity = 0.6
        self.max_delta_omega = np.pi/2
        self.turtlebot_radius = 0.07
        self.robot_names = robot_names

        # make an endless cycling iterator for the patterns
        self.pattern_names = cycle(patterns.keys())
        self.goal_positions = patterns

        # init subscriptions and publishers
        self.odom_subscriptions = self.init_subscriptions(robot_names)
        self.twist_publishers = self.init_twist_publishers(robot_names)

        # flag for stopping the rate
        self.stop_rate = False

    def shutdown_handler(self, sig, frame):
        """
        Handler that is executed when the
        node gets shut down.
        Should write the timings for the robots
        to a file
        """
        data = self.positions

        # remove timestamps from data
        timestamps = data.pop('timestamp')

        # construct multi level index
        columns = pd.MultiIndex.from_tuples(
            [(group, col_name) for group in data if group != 'timestamp' for
             col_name in ['x', 'y', 'w']],
            names=['Robot', 'Coordinate']
        )

        # prepare data
        arrays = []
        for val in data.values():
            arrays.append(np.array(val))
        data_transposed = np.hstack(arrays)

        # build dataframe
        df = pd.DataFrame(data_transposed, columns=columns, index=timestamps)

        # set up timing logfile
        current_timestamp = datetime.now().strftime("%Y_%m_%d:%H_%M_%S")
        timing_file = '{}.csv'.format(current_timestamp)
        directory = Path(BASE_DIR, 'timing', self.get_name())
        os.makedirs(directory, exist_ok=True)
        timing_path = Path(directory, timing_file)

        # save timings
        self.get_logger().info("Writing timing file to {}".format(timing_path))
        df.to_csv(timing_path)

        # shut down node
        self.destroy_node()
        rclpy.shutdown()


    def log_positions(self):
        """
        Save current robot
        positions
        """
        self.positions['timestamp'].append(self.simulation_time)
        for robot in self.robots.values():
            self.positions[robot.name].append(
                [
                    robot.position.x,
                    robot.position.y,
                    robot.w
                ]
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
        self.simulation_time = time_msg.clock.sec + time_msg.clock.nanosec / 1e9

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
