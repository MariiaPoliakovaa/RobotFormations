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

    def in_deadzone(self, other) -> bool:
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
            twist_msg.linear.x = self.preferred_velocity()
        return twist_msg

    def preferred_velocity(self):
        """
        Computes the preferred speed for the provided robot

        I used the same formula as artistic pattern generation,
        without computing the velocity and using the 
        maximum base speed as we don't need to speed up
        """
        # choose Vp to be the maximal speed
        Vp = self.max_linear_speed

        K = 1.3

        # calculate preferred speed
        preferred_speed = Vp * np.min(np.array([
            1,
            (self.position.distance_to(self.target) / K)
        ]))

        return preferred_speed

