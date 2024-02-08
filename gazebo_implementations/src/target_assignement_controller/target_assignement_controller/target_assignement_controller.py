import rclpy

from copy import deepcopy

from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from typing import List, Dict

from .turtlebot import Turtlebot, Point


class TargetAssignementController(Node):
    def __init__(self, robot_names: List[str], targets):
        super().__init__("target_assignement_controller")
        max_angular_speed = 0.5
        max_linear_speed = 0.5

        self.robot_names = robot_names
        self.robots = {name: Turtlebot(name,
                                       max_angular_speed,
                                       max_linear_speed)
                       for name in robot_names}

        self.odom_subscriptions = self.init_subscriptions(robot_names)
        self.twist_publishers = self.init_twist_publishers(robot_names)

        # this is just to initially test shit
        for robot_name, target in zip(robot_names, targets):
            robot = self.robots[robot_name]
            robot.target = target

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
        # get the robot
        robot = self.robots[robot_name]

        # get the publisher corresponding to robot
        twist_publisher = self.twist_publishers[robot_name]

        # read robot positions
        robot.position.x = odom_msg.pose.pose.position.x
        robot.position.y = odom_msg.pose.pose.position.y

        twist_msg = Twist()

        # if the robots target is set and its not reached its target
        if robot.target is not None and not robot.reached_target():

            # rotate it towards its target
            twist_msg, correctly_oriented = robot.rotate_to_target(odom_msg, twist_msg)

            self.get_logger().info(f'{robot_name} moving: {robot.run}',
                                   throttle_duration_sec=2)

            # if it aim towards its target start moving
            if correctly_oriented:
                robot.start_moving()

            # if the robot is supposed to run and has not reached its target
            if robot.run == True:
                twist_msg = robot.set_linear_speed(twist_msg)

        # publish twist message
        twist_publisher.publish(twist_msg)




def main():
    rclpy.init()

    # Initialize target positions
    target_positions = [
        Point(3,-3),
        Point(-3, 3)
    ]

    # Initialize target assignement controller
    robots = ['Tony', 'Jacky']
    tac = TargetAssignementController(robots, target_positions)

    rclpy.spin(tac)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
