import rclpy
import csv
import threading
import ipdb
import rvo2
import yaml
import json
import signal
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
from base_controller.base_controller import BaseController

from .artistic_pattern_gen import create_rectangle, create_circle,create_triangle, goal_generation, euclidean_dist, create_star
from .turtlebot import ArtisticTurtleBot as Turtlebot


class ArtisticPatternController(BaseController):
    def __init__(self, robot_names: List[str], goal_positions: Dict[str, Polygon]):
        """
        :param robot_names : A list of robot names
        :param patterns    : A dict with the names and polygons of patterns
        """
        super().__init__("artistic_pattern_controller", robot_names, goal_positions)
        self.targets_set = False
        self.sim = None

        # init turtlebot objects
        self.robots = {
            name: Turtlebot(name,
                            self.max_angular_speed,
                            self.max_linear_speed,
                            self.max_delta_velocity,
                            self.max_delta_omega
            ) for name in robot_names}

        # init targets for robots
        self.robot_goals = {name: None for name in robot_names}


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


    def run(self):
        # define rate at which to send messages
        update_rate = 5 # /s
        real_time_factor = 0.8
        sim_update_rate = update_rate * real_time_factor

        # start rate
        self.rate = self.create_rate(sim_update_rate)

        while rclpy.ok():
            if self.all_positions_assigned():
                t1 = time.time()
                self.log_positions()

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

    # set signal handler for shutdown
    signal.signal(signal.SIGINT, controller.shutdown_handler)

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(controller, ), daemon=True)
    thread.start()

    controller.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
