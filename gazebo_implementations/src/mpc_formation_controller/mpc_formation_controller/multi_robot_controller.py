import rclpy
import signal
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
from .multi_robot_optimization import Solver


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
        self.max_angular_speed = 0.1
        self.max_linear_speed = 0.12
        self.max_delta_velocity = 1.5
        self.max_delta_omega = np.pi/2
        self.min_dist = 0.0
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

        # init subscriptions
        self.odom_subscriptions = self.init_odom_subscriptions(robot_names)
        self.time_subscription = self.init_time_subscription()
        self.twist_publishers = self.init_twist_publishers(robot_names)


    def init_mpc_solver(self, dt: float):
        self.solver = Solver(
            len(self.robots),
            self.min_dist,
            self.max_linear_speed,
            self.max_angular_speed,
            self.max_delta_velocity,
            self.max_delta_omega,
            dt,
            self.T,
            self.get_logger()
        )


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
                (self.last_sim_time_values[0] - self.last_sim_time_values[4])/
                (self.last_real_time_values[0] - self.last_real_time_values[4])
            )
        #self.get_logger().info("Rt factor: {:.2f}".format(self.gazebo_real_time_factor))

    def assign_targets(self):
        """
        Generates target positions from the pattern and assigns
        those to the robots

        :param pattern : The pattern used to generate target positions
        """
        # use next pattern
        pattern_name = next(self.pattern_names)
        self.get_logger().info("New pattern: {}".format(pattern_name))
        self.pattern_name = pattern_name
        goal_positions = self.goal_positions[pattern_name]

        # create distance matrix
        n = len(self.robot_names)
        distances = np.zeros((n, n))
        robot_names = []
        for i, (robot_name, robot) in enumerate(self.robots.items()):
            robot_names.append(robot_name)
            for j, goal_position in enumerate(goal_positions):
                dist = np.sqrt( (robot.state[0] - goal_position[0])**2 +
                               (robot.state[1] - goal_position[1])**2 )
                distances[i][j] = dist**2

        # do target assignement
        assigned_positions = linear_sum_assignment(distances, maximize=False)

        for robot_idx, goal_idx in zip(assigned_positions[0], assigned_positions[1]):
            robot_name = robot_names[robot_idx]
            robot = self.robots[robot_name]
            goal_position = goal_positions[goal_idx]

            robot.set_target(
                goal_position[0],
                goal_position[1],
                0 # TODO: implement other angles
            )

    def all_positions_assigned(self):
        """
        Returns True if all turtlebots know their
        positions, else False
        """
        positions_set = True
        for robot in self.robots.values():
            if robot.state is None:
                positions_set = False
        return positions_set

    def targets_assigned(self):
        """
        Returns True if all robots have their target assigned,
        else False
        """
        if not self.all_positions_assigned():
            return

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
            if not robot.reached_target():
                done = False
        return done

    def reset_robots(self):
        """
        Unsets the targets for all robots
        """
        for robot in self.robots.values():
            robot.target = None
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

    def send_optimal_controll(self):
        """
        Computes and sends the most optimal controll inputs
        for all robots
        """
        robots = list(self.robots.values())
        current_states = [robot.state for robot in robots]
        target_states = [robot.target for robot in robots]

        #self.get_logger().info("\n" + str(current_states) + "\n" + str(target_states))

        if None in current_states or self.solver is None:
            # only execute after the robot state has been set
            # and the solver initialized
            return

        # compute optimal u
        optimal_u = self.solver.solve(
            current_states,
            target_states
        )

        # send optimal u to each robot
        for i, u in enumerate(optimal_u):
            # cast optimal u to numpy
            u = np.array(u).flatten()

            twist_msg = Twist()

            # set linear speed
            twist_msg.linear.x = u[0]

            # set angular speed
            twist_msg.angular.z = u[1]

            # send optimal controll to robot
            robot = robots[i]
            publisher = self.twist_publishers[robot.name]
            publisher.publish(twist_msg)

    def run(self):
        # define rate at which to send messages
        update_rate = 2 # /s

        # Init mpc_solver
        self.init_mpc_solver(1/update_rate)

        # start rate
        self.rate = self.create_rate(update_rate)

        while rclpy.ok():
            if self.all_positions_assigned():
                t1 = time.time()

                if not self.targets_assigned():
                    self.assign_targets()
                else:
                    self.send_optimal_controll()

                if self.pattern_achieved():
                    self.reset_robots()

                # warn if timing constraint violated
                t2 = time.time()
                td = t2 - t1
                self.get_logger().info("td: {:.2f}".format(td))
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
    T = 20
    controller = MultiRobotController(robots, goal_positions, T)

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(controller, ), daemon=True)
    thread.start()

    controller.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
