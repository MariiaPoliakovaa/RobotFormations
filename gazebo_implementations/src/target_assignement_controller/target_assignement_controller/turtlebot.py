import numpy as np
import math
import transforms3d as tf3d
from tf_transformations import euler_from_quaternion

from turtlebot_controller.turtlebot_controller import Turtlebot, Point

class TargetAssignementTurtlebot(Turtlebot):
    def __init__(self, name: str, max_angular_speed: float,
                 max_linear_speed: float, deadzone_radius: float):
        super().__init__(name, max_angular_speed, max_linear_speed)
        self.priority = None
        self.paused = False
        self.deadzone_radius = deadzone_radius

    def set_priority(self, prio: float):
        """
        Sets the priority of the turtlebot
        """
        self.priority = prio

    def pause(self):
        """
        Pauses the turtlebot
        """
        self.paused = True

    def unpause(self):
        """
        Unpauses the turtlebot
        """
        self.paused = False

    def in_deadzone(self, other):
        """
        Returns True if another turtlebot is within
        the deadzone radius, else False
        """
        if self.position.distance_to(other.position) < self.deadzone_radius:
            return True
        else:
            return False

    def set_linear_speed(self, twist_msg):
        """
        Sets the velocity of the turtlebot
        """
        if self.run and not self.paused:
            twist_msg.linear.x = self.linear_speed
        return twist_msg
