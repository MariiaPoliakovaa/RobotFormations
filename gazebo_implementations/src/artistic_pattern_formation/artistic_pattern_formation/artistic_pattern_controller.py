import rclpy
import ipdb
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
from geometry_msgs.msg import Twist

from typing import List, Dict

from .turtlebot import Turtlebot, Point
from .artistic_pattern_gen import create_rectangle, create_circle, create_triangle, goal_generation
from multi_agent_path_planning.centralized.cbs.cbs import cbs as path_finding


def round_floats(obj, precision=3):
    """
    I just need this for debugging,
    it rounds all floats in a dict
    or list
    """
    if isinstance(obj, float):
        return float(round(obj, precision))
    elif isinstance(obj, dict):
        return {k: round_floats(v, precision) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [round_floats(item, precision) for item in obj]
    else:
        return obj


class ArtisticPatternController(Node):
    def __init__(self, robot_names: List[str], patterns: List[Polygon]):
        """
        :param robot_names : A list of robot names
        :param patterns    : The patterns that are used for the formations
        """
        super().__init__("artistic_pattern_controller")
        max_angular_speed = 0.5
        max_linear_speed = 0.5
        self.targets_set = False
        self.current_step = 0

        # make an endless cycling iterator for the patterns
        self.patterns = cycle(patterns)

        # init turtlebot objects
        self.robot_names = robot_names
        self.robots = {
            name: Turtlebot(name,
                            max_angular_speed,
                            max_linear_speed
            ) for name in robot_names}

        # init targets for robots
        self.robot_goals = {name: None for name in robot_names}

        # init subscriptions and publishers
        self.odom_subscriptions = self.init_subscriptions(robot_names)
        self.twist_publishers = self.init_twist_publishers(robot_names)


    def assign_targets(self):
        """
        Generates target positions from the pattern and assigns
        those to the robots

        :param pattern : The pattern used to generate target positions
        """
        # use next pattern
        pattern = next(self.patterns)

        # generate goal positions
        #self.get_logger().info("Goal generation")
        #goal_positions = goal_generation(pattern, len(self.robots))
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

        # For debugging: write robot positions and goal positions to yml
        agents = []
        for name, robot in self.robots.items():
            agents.append({
                'start': [robot.position.x, robot.position.y],
                'goal': [self.robot_goals[name].x, self.robot_goals[name].y],
                'name': name
            })
        params = {
            'agents': agents,
            'map': {
                'dimensions': [5,5],
                'obstacles': None
            }
        }
        file = "/home/mariia/Documents/Masterarbeit/Implementation_example/gazebo_implementations/src/multi_agent_path_planning/multi_agent_path_planning/centralized/cbs/input_debug.yaml"
        with open(file, 'w') as f:
            yaml.safe_dump(round_floats(params), f)

        # Compute the path finding steps
        self.get_logger().info("Compute path finding steps")
        path_finding_steps = path_finding(self.robots, self.robot_goals)

        self.get_logger().info(path_finding_steps)

        # set path finding steps for each robot
        for robot_name, robot in self.robots.items():
            robot.set_path_finding_steps(path_finding_steps[robot_name])


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


    def path_finding_step_complete(self) -> bool:
        """
        Returns true if every robot has reached it's
        current path finding goal
        """
        done = True
        for robot in self.robots.values():
            if not robot.reached_target():
                done = False
        return done


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
        for robot_name in self.robot_names:
            topic_name = f'/{robot_name}/odom'
            subscription = self.create_subscription(
                Odometry,
                topic_name,
                lambda msg, robot_name=robot_name: self.odom_callback(msg, robot_name),
                10
            )
            subscriptions.append(subscription)
        return subscriptions


    def odom_callback(self, odom_msg: Odometry, robot_name: str):
        """
        Odometry callback to receive pose from the turtlebot and
        send a twist message based on the turtlebots state
        """

        self.get_logger().info("===========================")
        self.get_logger().info("Received odom message for {}".format(robot_name))
        # if positions are not yet assigned and all robot positions were received
        if not self.targets_set and self.all_positions_assigned():
            self.get_logger().info("Assigning targets")
            self.assign_targets()
            self.targets_set = True

        # if all robots have reached their current path finding goal,
        # increment the current frame
        if self.all_positions_assigned():
            if self.path_finding_step_complete():
                self.get_logger().info("Incrementing step")
                self.current_step += 1

        # get the robot
        robot = self.robots[robot_name]

        # set the robots current path finding goal
        if self.all_positions_assigned():
            self.get_logger().info("Setting path finding step")
            robot.set_path_finding_step(self.current_step)

        # get the publisher corresponding to robot
        twist_publisher = self.twist_publishers[robot_name]

        # read robot positions
        robot.position = Point(
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y
        )

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
