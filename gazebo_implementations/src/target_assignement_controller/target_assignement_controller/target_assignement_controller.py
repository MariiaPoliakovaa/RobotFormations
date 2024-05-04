import rclpy
import json
import os
import csv
import time
import numpy as np

from datetime import datetime
from copy import deepcopy
from pathlib import Path

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, LivelinessPolicy
from rosgraph_msgs.msg import Clock
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from itertools import cycle
from typing import List, Dict

from turtlebot_controller.turtlebot_controller import Point
from base_controller.base_controller import BaseController
from .turtlebot import TargetAssignementTurtlebot as Turtlebot
from .target_assignement import target_assignement_strategy

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


class TargetAssignementController(BaseController):
    def __init__(self, robot_names: List[str], goal_positions: Dict[str, list]):
        super().__init__("target_assignement_controller", robot_names)
        max_angular_speed = 2.84
        max_linear_speed = 0.22
        turtlebot_radius = 0.07
        deadzone_radius = (2 * turtlebot_radius) * 1.4

        self.targets_set = False
        self.seconds = 0
        # set up timing logfile
        current_timestamp = datetime.now().strftime("%Y_%m_%d:%H_%M_%S")
        timing_file = 'target_assignement_{}.csv'.format(current_timestamp)
        self.timing_path = Path(BASE_DIR, 'timing', timing_file)

        self.pattern_names = cycle(goal_positions.keys())
        self.goal_positions = goal_positions

        # init turtlebot objects
        self.robot_names = robot_names
        self.robots = {name: Turtlebot(name,
                                       max_angular_speed,
                                       max_linear_speed,
                                       deadzone_radius)
                       for name in robot_names}

        # init subscriptions and publishers
        self.odom_subscriptions = self.init_subscriptions(robot_names)
        self.twist_publishers = self.init_twist_publishers(robot_names)

    def all_positions_set(self):
        """
        Returns True if all turtlebots have their
        positions set, else False
        """
        positions_set = True
        for robot in self.robots.values():
            if robot.position is None:
                positions_set = False
        return positions_set

    def assign_targets_prios(self):
        """
        Use Use target assignement startegy to assign
        targets to the turtlebots
        """
        # get all the turtlebots
        turtlebots = list(self.robots.values())

        # get the current pattern
        pattern_name = next(self.pattern_names)
        self.pattern_name = pattern_name
        goal_positions = self.goal_positions[pattern_name]

        # assign targets to the turtlebots
        prio_direction = 'increasing'
        turtlebots_assigned = target_assignement_strategy(
            turtlebots,
            goal_positions,
            prio_direction
        )

        # reshape into dictionary
        turtlebots_assigned = {robot.name: robot for
                              robot in turtlebots_assigned}

        return turtlebots_assigned


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
        for robot_name in self.robot_names:
            topic_name = f'/{robot_name}/odom'
            subscription = self.create_subscription(
                Odometry,
                topic_name,
                lambda msg, robot_name=robot_name: self.odom_callback(msg, robot_name),
                10
            )
            subscriptions.append(subscription)

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

    def robots_correctly_oriented(self):
        """
        Returns true if all robots are correctly oriented
        """
        correctly_oriented = True

        for robot in self.robots.values():
            if not robot.correctly_oriented:
                correctly_oriented = False

        return correctly_oriented

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

    def check_deadzones(self, robot: Turtlebot):
        """
        Checks if there is another robot in this
        robots deadzone.
        If there is one, the turtlebot with the lower
        priority is stopped
        """
        robot.unpause()

        for other_robot in self.robots.values():
            if robot.in_deadzone(other_robot) and (robot.name != other_robot.name):
                dist = robot.position.distance_to(other_robot.position)

                if robot.priority < other_robot.priority:
                    robot.pause()
                else:
                    other_robot.pause()

    def log_timing(self):
        """
        Writes the current timing of the current
        formation to csv file
        """
        data = [self.pattern_name, "{:.2f}".format(self.seconds)]
        append_to_csv(self.timing_path, data)

    def odom_callback(self, odom_msg: Odometry, robot_name: str):
        """
        Odometry callback to receive pose from the turtlebot and
        send a twist message based on the turtlebots state
        """
        # if positions are not yet assigned and all robot positions were
        # received
        if not self.targets_set and self.all_positions_set():
            self.robots = self.assign_targets_prios()
            self.targets_set = True
            self.log_timing()

        # get the robot
        robot = self.robots[robot_name]

        # get the publisher corresponding to robot
        twist_publisher = self.twist_publishers[robot_name]

        # read robot positions
        robot.position = Point(
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y
        )

        # Save initial position
        if robot.initial_position is None:
            robot.initial_position = robot.position

        twist_msg = Twist()

        # if the robots target is set and its not reached its target
        if robot.target is not None and not robot.reached_target():

            # rotate it towards its target
            twist_msg = robot.rotate_to_target(odom_msg, twist_msg)

            # if it aim towards its target start moving
            if self.robots_correctly_oriented():
                robot.start_moving()

            # check if there is another robot in this robots deadzone
            # if there is, stop the one with the lower priority
            if self.all_positions_set():
                self.check_deadzones(robot)

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

def main():
    rclpy.init()

    # get robots
    robots = json.loads(os.environ.get("ROBOTS"))
    robots = [robot['name'] for robot in robots]

    # Initialize target positions
    goal_positions = os.environ.get("TARGET_POSITIONS")

    if goal_positions is not None:
        goal_positions = json.loads(goal_positions)
    else:
        raise ValueError("No target patterns in environment")

    # Initialize target assignement controller
    tac = TargetAssignementController(robots, goal_positions)

    rclpy.spin(tac)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
