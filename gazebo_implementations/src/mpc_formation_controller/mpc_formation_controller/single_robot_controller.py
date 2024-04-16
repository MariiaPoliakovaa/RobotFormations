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


class SingleRobotController(Node):
    def __init__(self, target: List[float], T: int):
        """
        :param target : target for the robot
        :param T      : Forcast horizon for optimization
        """
        super().__init__("single_robot_mpc_controller")
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

        # init robot
        self.robot = Turtlebot(
            'robot1',
            self.max_angular_speed,
            self.max_linear_speed
        )

        # set target for robot
        self.robot.set_target(
            target[0],
            target[1],
            target[2]
        )

        self.solver = Solver(
            self.max_linear_speed,
            self.max_angular_speed,
            self.max_delta_velocity,
            self.max_delta_omega,
            1/self.update_rate,
            self.T
        )

        # init odometry subscription
        self.odom_sub = self.init_odom_subscription()

        # init time sub
        self.time_sub = self.init_time_subscription()

        # init twist publisher
        self.pub = self.init_twist_publisher()

    def init_twist_publisher(self):
        """
        Initializes the Twist message publisher for the
        turtlebot.

        """
        robot_name = self.robot.name
        topic_name = f'/{robot_name}/cmd_vel'
        publisher = self.create_publisher(
            Twist,
            topic_name,
            10
        )
        return publisher

    def init_odom_subscription(self):
        # subscribe to odometry messages for each robot
        robot_name = self.robot.name
        topic_name = f'/{robot_name}/odom'
        odom_subscription = self.create_subscription(
            Odometry,
            topic_name,
            lambda msg, robot_name=robot_name: self.odom_callback(msg, robot_name),
            10
        )
        return odom_subscription

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

    def odom_callback(self, odom_msg: Odometry, robot_name: str):
        """
        The odometry callback updates the current position
        of the robot
        """
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
        self.get_logger().info("State: {:.2f}-{:.2f}-{:.2f}".format(
            x, y, w
        ))

        self.robot.set_state(x, y, w)

    def send_optimal_controll(self):
        """
        Computes the most optimal controll signal
        by solvin the for the minimum of the
        cost function in:
        "https://braca51e.medium.com/navigation-
        using-model-predictive-control-with-turtlebot3-7d7c55ab22a6"
        """
        current_state = self.robot.state
        target_state = self.robot.target

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
        self.pub.publish(twist_msg)

    def run(self):
        self.rate = self.create_rate(self.update_rate)
        self.current_real_time_factor = self.gazebo_real_time_factor

        while rclpy.ok():
            if self.robot.state is not None:
                #if abs(self.gazebo_real_time_factor - self.current_real_time_factor) > 0.05:
                #    self.current_real_time_factor = self.gazebo_real_time_factor
                #    new_rate = self.update_rate * self.current_real_time_factor
                #    self.rate = self.create_rate(new_rate)


                self.get_logger().info("State: {:.2f}-{:.2f}-{:.2f}".format(
                    self.robot.state[0],
                    self.robot.state[1],
                    self.robot.state[2]
                ))
                self.get_logger().info("Real time factor: {:.2f}".format(
                    self.gazebo_real_time_factor
                ))
                self.send_optimal_controll()
            self.rate.sleep()

def main():
    rclpy.init()

    # Initialize target assignement controller
    T = 50
    goal_position = [-1, -1, -np.pi/2]
    controller = SingleRobotController(goal_position, T)

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(controller, ), daemon=True)
    thread.start()

    controller.run()
    #rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
