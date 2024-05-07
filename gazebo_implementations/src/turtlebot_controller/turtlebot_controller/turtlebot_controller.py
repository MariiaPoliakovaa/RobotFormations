import numpy as np
import math
import ipdb
import transforms3d as tf3d
from tf_transformations import euler_from_quaternion
from typing import List
from rclpy.logging import get_logger

logger = get_logger('turtlebot_logger')

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance_to(self, other) -> float:
        """
        Calculates the euclidean distance to another Point

        :param other : The other point

        :returns     : Distance to other point
        """
        return np.sqrt( (self.x - other.x)**2 + (self.y - other.y)**2 )

    def __repr__(self):
        return "({:.2f}, {:.2f})".format(self.x, self.y)

    def __str__(self):
        return "({:.2f}, {:.2f})".format(self.x, self.y)


class Turtlebot:
    def __init__(self,
                 name: str,
                 max_angular_speed: float,
                 max_linear_speed: float,
                 max_delta_vel = 0.6,
                 max_delta_omega = 1.5):
        self.name = name
        self.position = None
        self.initial_position = None
        self.priority = None
        self.target = None
        self.run = False
        self.correctly_oriented = False

        self.angular_speed = max_angular_speed
        self.linear_speed = max_linear_speed
        self.max_linear_speed = max_linear_speed
        self.angle_threshold = 0.05
        self.distance_threshold = 0.08

    def reached_target(self):
        """
        Checks wether the turtlebot has reached its target
        """
        return self.position.distance_to(self.target) < self.distance_threshold

    def set_target(self, target: Point):
        self.target = target

    def start_moving(self):
        self.run = True

    def stop_moving(self):
        self.run = False

    def rotate_to_target(self, twist_msg):
        """
        Adds a twist element to <twist_msg>
        to aim the turtlebot towards its target.
        """

        # get angle to target
        angle_to_target = math.atan2(
            self.target.y - self.position.y,
            self.target.x - self.position.x
        ) % (2*np.pi)

        # calculate how far to move
        angle_difference = angle_to_target - self.w
        angular_speed = self.angular_speed * angle_difference/np.pi
        twist_msg.angular.z = angular_speed

        if abs(angle_difference) > self.angle_threshold:
            # reduce angular speed based on angle difference
            correctly_oriented = False
            self.stop_moving()
        else:
            correctly_oriented = True

        self.correctly_oriented = correctly_oriented
        return twist_msg

    def set_linear_speed(self, twist_msg):
        """
        Sets the velocity of the turtlebot
        """
        if self.run == True:
            twist_msg.linear.x = self.linear_speed
        return twist_msg


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

        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # We need to subtract pi because the euler from quaternion
        # function returns values in [0, 2*pi] and we want
        # to have values in [-pi, pi]
        return yaw % (2*np.pi) #- np.pi
