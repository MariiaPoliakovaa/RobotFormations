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

        """
        if initial_position.distance_to(goal) < initial_dist_threhold:
            factor = 1 - (initial_dist_threhold - initial_position.distance_to(goal)) / initial_dist_threhold

            # reduce Vp the closer the initial position is to the goal
            Vp *= factor
        """

        # Make vectors for position and goal
        position_v = np.array([position.x, position.y])
        goal_v = np.array([goal.x, goal.y])

        #logger.info("Position: "+ str(position_v))
        #logger.info("Goal: "+ str(goal_v))

        # calculate K
        #K = initial_position.distance_to(goal)
        K = 0.5

        # calculate preferred speed
        preferred_speed = Vp * np.min(np.array([
            1,
            (position.distance_to(goal) / K)
        ]))

        # calculate preferred velocity
        preferred_velocity = preferred_speed * (goal_v - position_v) / position.distance_to(goal)
        preferred_velocity = (preferred_velocity[0], preferred_velocity[1])

        #preferred_velocity = self.reduce_rotation(preferred_velocity)

        return preferred_velocity


    def reduce_rotation(self, velocity: Tuple[float, float]) -> Tuple[float, float]:
        """
        Reduces the rotation of the preferred_velocity the closer 
        the rotation of the robot is to the target rotation.
        This should prevent overshooting the rotation and lead
        to less oscilation in the path

        :param velocity : The preferred velocity

        :returns        : The preferred velocity with the rotation reduced
        """
        # get rotation and speed from velocity vector
        preferred_rotation = np.arctan2(velocity[1], velocity[0])
        preferred_speed = np.linalg.norm(np.array(velocity))
        logger.info("Preferred rotation: {:.2f}".format(preferred_rotation))

        preferred_rotation *= 0.2

        # reconstruct updated velocity vector
        vel_x = preferred_speed * np.cos(preferred_rotation)
        vel_y = preferred_speed * np.sin(preferred_rotation)

        #return (vel_x, vel_y)
        return velocity


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
