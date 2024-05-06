import rclpy
import ipdb
import threading
import signal
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
        self.deadzone_radius = 0.35

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
        self.pattern_name = next(self.pattern_names)
        goal_positions = self.goal_positions[self.pattern_name]

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

        self.robots = turtlebots_assigned

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
                self.get_logger().info(f"{robot.name} in deadzone of\
                                       {other_robot.name}")

                if robot.priority < other_robot.priority:
                    robot.pause()
                else:
                    other_robot.pause()

    def send_controll_signals(self):
        """
        Sends cmd_vel messages to the Robots
        """
        for robot in self.robots.values():
            twist_publisher = self.twist_publishers[robot.name]
            twist_msg = Twist()

            # if the robots target is set and its not reached its target
            if robot.target is not None and not robot.reached_target():

                # rotate it towards its target
                twist_msg = robot.rotate_to_target(twist_msg)

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
                self.log_positions()

                if not self.targets_assigned():
                    self.assign_targets_prios()
                else:
                    # send controll signals as in simulation
                    self.send_controll_signals()

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

    # Initialize target positions
    goal_positions = os.environ.get("TARGET_POSITIONS")

    if goal_positions is not None:
        goal_positions = json.loads(goal_positions)
    else:
        raise ValueError("No target patterns in environment")

    # Initialize target assignement controller
    controller = TargetAssignementController(robots, goal_positions)

    # set signal handler for shutdown
    signal.signal(signal.SIGINT, controller.shutdown_handler)

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(controller, ), daemon=True)
    thread.start()

    controller.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
