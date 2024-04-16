import rclpy
import threading
import csv
import ipdb
import rvo2
import yaml
import json
import time
import os
import logging
import numpy as np
import casadi as cas

from copy import deepcopy
from pathlib import Path
from shapely import Polygon
from itertools import cycle
from scipy.optimize import linear_sum_assignment
from datetime import datetime
from collections import deque

from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

from typing import List, Dict

from .turtlebot import MPCTurtleBot as Turtlebot
from .optimization import Solver


class MultiRobotController(Node):
    def __init__(self,
                 robot_names: List[str],
                 goal_positions: Dict[str, Polygon],
                 T: int):
        """
        :param robot_names   : guess what? THe rObOT nAmEs
        :param goal_positions: goal positions
        :param T             : Forcast horizon
        """
        super().__init__("multi_robot_mpc_controller")
        self.update_rate = 5
        self.max_angular_speed = 0.5
        self.max_linear_speed = 0.12
        self.max_delta_velocity = 1.5
        self.max_delta_omega = np.pi/2
        self.last_sim_time_values = deque(maxlen=5)
        self.last_real_time_values = deque(maxlen=5)
        self.gazebo_real_time_factor = 1
        self.T = T
        self.dt = None
        self.dts = []
        self.solver = None

        # make an endless cycling iterator for the patterns
        self.pattern_names = cycle(goal_positions.keys())
        self.goal_positions = goal_positions

        # make an endless cycling iterator for the patterns
        self.pattern_names = cycle(goal_positions.keys())
        self.goal_positions = goal_positions

        # init turtlebot objects
        self.robot_names = robot_names
        self.robots = {
            name: Turtlebot(name,
                            self.max_angular_speed,
                            self.max_linear_speed
            ) for name in robot_names}

        # init MPC solver
        self.solver = Solver(
            self.max_linear_speed,
            self.max_angular_speed,
            self.max_delta_velocity,
            self.max_delta_omega,
            1/self.update_rate,
            self.T
        )

        # init subscriptions
        self.odom_subscriptions = self.init_odom_subscriptions(robot_names)
        self.time_subscription = self.init_time_subscription()
        self.twist_publishers = self.init_twist_publishers(robot_names)

    def init_twist_publishers(self, robot_names: List[str]) -> Dict[str, Publisher]:
        """
        Initializes the Twist message publisher for each
        turtlebot.

        :param robot_names : List of names of the turtlebots
        :returns           : Dictionary with the robot names
                             as keys and the publishers as values
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

    def init_odom_subscriptions(self, robot_names: List[str]) -> Dict[str, Subscription]:
        """
        Initializes the subscriptions to the odometry message
        for each turtlebot.

        :param robot_names: List of names of the turtlebots
        :returns           : Dictionary with the robot names
                             as keys and the subscriptions
                             as values
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

    def init_time_subscription(self):
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

        return time_subscription

    def timer_callback(self, time_msg):
        """
        Computes the real time factor of
        the gazebo simulation
        """
        gazebo_time_ns = time_msg.clock.sec * 1e9 + time_msg.clock.nanosec
        real_time_ns = time.time_ns()

        self.last_sim_time_values.append(gazebo_time_ns)
        self.last_real_time_values.append(real_time_ns)

        if len(self.last_sim_time_values) >= 5:
            self.gazebo_real_time_factor = (
                (self.last_sim_time_values[4] - self.last_sim_time_values[0])/
                (self.last_real_time_values[4] - self.last_real_time_values[0])
            )

    def assign_targets(self):
        """
        Generates target positions from the pattern and assigns
        those to the robots

        :param pattern : The pattern used to generate target positions
        """
        t1 = time.time()
        # use next pattern
        pattern_name = next(self.pattern_names)
        self.pattern_name = pattern_name
        goal_positions = self.goal_positions[pattern_name]

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

    def reset_robots(self):
        """
        Unsets the targets for all robots
        """
        for robot in self.robots.values():
            robot.target = None
            robot.stop_moving()
            robot.initial_position = None
        self.targets_set = False

    def odom_callback(self, odom_msg: Odometry, robot_name: str):
        """
        The odometry callback updates the current position
        of the robot
        """
        # get the robot
        robot = self.robots[robot_name]

        # get x and y pos
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        # transform orientation to euler format
        orientation = odom_msg.pose.pose.orientation
        quaternion = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ]

        roll, pitch, yaw = euler_from_quaternion(quaternion)
        w = yaw

        robot.set_state(x, y, w)

    def send_optimal_controll(self, robot):
        """
        Computes the most optimal controll signal
        by solvin the for the minimum of the
        cost function in:
        "https://braca51e.medium.com/navigation-
        using-model-predictive-control-with-turtlebot3-7d7c55ab22a6"
        """
        current_state = robot.state
        target_state = robot.target

        if current_state is None or self.solver is None:
            # only execute after the robot state has been set
            return

        # compute optimal u
        optimal_u = self.solver.solve(
            current_state,
            target_state
        )

        # cast optimal u to numpy
        u = np.array(optimal_u).flatten()

        twist_msg = Twist()

        # set linear speed
        twist_msg.linear.x = u[0]

        # set angular speed
        twist_msg.angular.z = u[1]

        # send optimal controll to robot
        publisher = self.twist_publishers[robot.name]
        publisher.publish(twist_msg)

    def run(self):
        self.rate = self.create_rate(self.update_rate)

        while rclpy.ok():
            for robot in self.robots.values():
                if self.robot.state is not None:
                    self.send_optimal_controll(robot)
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
    T = 50
    controller = MultiRobotController(robots, goal_positions, T)

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(controller, ), daemon=True)
    thread.start()

    controller.run()
    #rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
