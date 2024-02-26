import rclpy
import time
import numpy as np

from copy import deepcopy

from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from typing import List, Dict

from .turtlebot import Turtlebot, Point
from .target_assignement import target_assignement_strategy


class TargetAssignementController(Node):
    def __init__(self, robot_names: List[str], targets):
        super().__init__("target_assignement_controller")
        max_angular_speed = 0.5
        max_linear_speed = 0.5
        deadzone_radius = 0.5 # I need to work out a useful value for this

        self.targets_set = False
        self.targets = targets

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


    def assign_targets_prios(self, targets: List[Point]):
        """
        Use Use target assignement startegy to assign
        targets to the turtlebots
        """
        # get all the turtlebots
        turtlebots = list(self.robots.values())

        # assign targets to the turtlebots
        prio_direction = 'increasing'
        turtlebots_assigned = target_assignement_strategy(
            turtlebots,
            targets,
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
        return subscriptions


    def check_deadzones(self, robot: Turtlebot):
        """
        Checks if there is another robot in this
        robots deadzone.
        If there is one, the turtlebot with the lower
        priority is stopped
        """
        for other_robot in self.robots.values():
            if robot.in_deadzone(other_robot) and (robot.name != other_robot.name):
                self.get_logger().info(f"{other_robot.name} in {robot.name}'s deadzone")

                dist = robot.position.distance_to(other_robot.position)
                self.get_logger().info(f"distance: {dist}")

                if robot.priority < other_robot.priority:
                    robot.stop_moving()
                else:
                    other_robot.stop_moving()


    def odom_callback(self, odom_msg: Odometry, robot_name: str):
        """
        Odometry callback to receive pose from the turtlebot and
        send a twist message based on the turtlebots state
        """
        # if positions are not yet assigned and all robot positions were
        # received
        if not self.targets_set and self.all_positions_set():
            self.robots = self.assign_targets_prios(self.targets)
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

        twist_msg = Twist()

        # if the robots target is set and its not reached its target
        if robot.target is not None and not robot.reached_target():

            # rotate it towards its target
            twist_msg, correctly_oriented = robot.rotate_to_target(odom_msg, twist_msg)

            # if it aim towards its target start moving
            if correctly_oriented:
                robot.start_moving()

            # check if there is another robot in this robots deadzone
            # if there is, stop the one with the lower priority
            self.check_deadzones(robot)

            # if the robot is supposed to run and has not reached its target
            if robot.run == True:
                twist_msg = robot.set_linear_speed(twist_msg)

        # publish twist message
        twist_publisher.publish(twist_msg)


def circle_positions(n: int, radius: float) -> List[Point]:
    """
    Generates evenly spaced points on a circle

    :param n      : number of points
    :param radius : radius of the circle

    :returns      : List of points on a circle
    """
    points = []

    for theta in np.linspace(0, 2*np.pi, n+1):
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        points.append(Point(x, y))

    return points[:-1]


def main():
    rclpy.init()

    # robot names
    robots = ['Tony', 'Jacky', 'Andreas', 'Susan', 'Pikachu', 'Brudi']
              #'DarthVader', 'Lisa', 'Gertrude']

    # Initialize target positions
    target_positions = circle_positions(len(robots), 1.3)

    # Initialize target assignement controller
    tac = TargetAssignementController(robots, target_positions)

    rclpy.spin(tac)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
