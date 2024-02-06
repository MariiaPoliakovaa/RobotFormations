import rclpy
import math
import numpy as np
import transforms3d as tf3d
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from .point import Point

class TurtlebotController(Node):
    def __init__(self, robot_name):
        super().__init__("{}_controller".format(robot_name))

        self.publisher_ = self.create_publisher(
            Twist,
            "/{}/cmd_vel".format(robot_name),
            10
        )

        self.subscription_ = self.create_subscription(
            Odometry,
            "/{}/odom".format(robot_name),
            self.odometry_callback,
            10
        )

        # Coordinates of turtlebot
        self.position = Point(None, None)

        # Initialize target as None
        self.target = None

        self.angular_speed = 0.5
        self.linear_speed = 0.2  # Adjust the linear speed as needed

        self.angle_threshold = 0.1
        self.distance_threshold = 0.1


    def set_target(self, target: Point):
        self.target = target


    def get_position(self):
        return self.position


    def odometry_callback(self, odom_msg):
        # extract turtlebot coordinates
        self.position.x = odom_msg.pose.pose.position.x
        self.position.y = odom_msg.pose.pose.position.y

        # calculate distance to target
        distance_to_target = self.position.distance_to(self.target)

        # initialize twist message
        twist_msg = Twist()

        # move turtlebot if it has not reached target and if target is set
        if (distance_to_target > self.distance_threshold) and (self.target is not None):

            # rotate the robot towards the target
            twist_msg, correctly_oriented = self.rotate_to_target(odom_msg, twist_msg)

            if correctly_oriented:
                # If the robot is correctly oriented: drive
                twist_msg.linear.x = self.linear_speed

        else:
            # Stop turtlebot
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        # publish twist message
        self.publisher_.publish(twist_msg)


    def rotate_to_target(self, odom_msg, twist_msg):
        """
        Adds a twist element to <twist_msg> to aim the turtlebot towards its
        target.
        """
        # get angle to target
        angle_to_target = math.atan2(
            self.target.y - self.position.y,
            self.target.x - self.position.x
        )

        # get current orientation
        current_orientation = self.get_orientation(odom_msg)

        # calculate how far to move
        angle_difference = angle_to_target - current_orientation

        if abs(angle_difference) > self.angle_threshold:
            angular_speed = self.angular_speed if angle_difference > 0 else -self.angular_speed
            twist_msg.angular.z = np.abs(angle_difference) * angular_speed
            correctly_oriented = False
        else:
            twist_msg.angular.z = 0.0
            correctly_oriented = True

        return twist_msg, correctly_oriented


    def get_orientation(self, msg):
        """
        Returns the orientation of the robot

        TODO: I am not quite sure if this works correctly... blupp
        """
        # Extract the orientation quaternion from the odometry message
        orientation = msg.pose.pose.orientation
        quaternion = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ]

        # Normalize the quaternion to handle small numerical errors
        quaternion /= tf3d.quaternions.qnorm(quaternion)

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        euler_angles = tf3d.euler.quat2euler(quaternion)
        #print(euler_angles)

        # extract yaw
        yaw = euler_angles[0]

        return yaw


def main():
    target = Point(-2, 2)

    rclpy.init()

    move_turtlebot = TurtlebotController('Tony')
    move_turtlebot.set_target(target)

    rclpy.spin(move_turtlebot)
    move_turtlebot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

