import math
import numpy as np
from typing import Tuple
from rclpy.logging import get_logger

from turtlebot_controller.turtlebot_controller import Turtlebot, Point

logger = get_logger('turtlebot_logger')

class ArtisticTurtleBot(Turtlebot):
    def preferred_velocity(self, goal):
        """
        Computes the preferred speed for the provided robot

        In the paper the velocity is computed. That means
        it is a vector. As we rotate the robot before moving,
        the speed should suffice.
        The second term is normalized and represents the direction
        vector.

        Problem: The robots speed up very quickly even if their
        target position is very close to them. This can cause them
        to overshoot their target or be unable to react in time to
        other robots. I propose reducing the robots speed if it is
        close to it's target.
        """
        position = self.position
        initial_position = self.initial_position

        # if the robot starts closer than this to the target
        # reduce its preferred velocity
        initial_dist_threhold = 1.0

        # choose Vp to be somewhat slower than the max speed
        Vp = 0.8 * self.max_linear_speed

        # Make vectors for position and goal
        position_v = np.array([position.x, position.y])
        goal_v = np.array([goal.x, goal.y])

        # calculate K
        #K = initial_position.distance_to(goal)
        K = 1.2

        # calculate preferred speed
        preferred_speed = Vp * np.min(np.array([
            1,
            (position.distance_to(goal) / K)
        ]))

        # calculate preferred velocity
        preferred_velocity = preferred_speed * (goal_v - position_v) / position.distance_to(goal)
        preferred_velocity = (preferred_velocity[0], preferred_velocity[1])

        return preferred_velocity

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
        #logger.info("Robot rotation: {:.2f}".format(self.w-np.pi))

        if abs(angle_difference) > self.angle_threshold:
            # reduce angular speed based on angle difference
            angular_speed = self.angular_speed * angle_difference/np.pi
            twist_msg.angular.z = angular_speed
            correctly_oriented = False
        else:
            correctly_oriented = True

        self.correctly_oriented = correctly_oriented
        return twist_msg
