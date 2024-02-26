import numpy as np
import math
import transforms3d as tf3d
from tf_transformations import euler_from_quaternion


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance_to(self, other) -> float:
        """
        Calculates the distance to another Point

        :param other : The other point

        :returns     : Distance to other point
        """
        return np.sqrt( (self.x - other.x)**2 + (self.y - other.y)**2 )

    def __repr__(self):
        return "({:.2f}, {:.2f})".format(self.x, self.y)

    def __str__(self):
        return "({:.2f}, {:.2f})".format(self.x, self.y)


class Turtlebot:
    def __init__(self, name: str, max_angular_speed: float,
                 max_linear_speed: float, deadzone_radius: float):
        self.name = name
        self.position = None
        self.priority = None
        self.target = None
        self.run = False

        self.angular_speed = max_angular_speed
        self.linear_speed = max_linear_speed
        self.angle_threshold = 0.1
        self.distance_threshold = 0.1

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

    def set_priority(self, prio: float):
        self.priority = prio

    def set_path_finding_steps(self, steps: List[dict]):
        """
        Sets the path finding steps for the turtlebot
        """
        self.path_finding_steps = {}
        for step in steps:
            self.path_finding_steps[step['t']] = Point(step['x'], step['y'])

    def set_path_finding_step(self, step: int):
        self.set_target(self.path_finding_steps[step])

    def rotate_to_target(self, odom_msg, twist_msg):
        """
        Adds a twist element to <twist_msg>
        to aim the turtlebot towards its target.
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
            twist_msg.angular.z = angular_speed
            correctly_oriented = False
        else:
            correctly_oriented = True

        return twist_msg, correctly_oriented


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

        return yaw
