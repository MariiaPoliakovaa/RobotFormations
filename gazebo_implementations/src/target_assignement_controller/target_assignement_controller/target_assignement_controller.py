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


class TargetAssignementController(BaseController):
    def __init__(self, robot_names: List[str], goal_positions: Dict[str, list]):
        super().__init__("target_assignement_controller", robot_names,
                         goal_positions)
        self.deadzone_radius = (2 * self.turtlebot_radius) * 1.4

        # init turtlebot objects
        self.robot_names = robot_names
        self.robots = {name: Turtlebot(name,
                                       self.max_angular_speed,
                                       self.max_linear_speed,
                                       self.deadzone_radius)
                       for name in robot_names}

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

    def odom_callback(self, odom_msg: Odometry, robot_name: str):
        """
        Odometry callback to receive pose from the turtlebot and
        send a twist message based on the turtlebots state
        """
        # if positions are not yet assigned and all robot positions were
        # received
        if not self.targets_assigned() and self.all_positions_assigned():
            self.robots = self.assign_targets_prios()
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
            if self.all_positions_assigned():
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
