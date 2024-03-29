import numpy as np

from turtlebot_controller.turtlebot_controller import Turtlebot, Point

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

        if initial_position.distance_to(goal) < initial_dist_threhold:
            factor = 1 - (initial_dist_threhold - initial_position.distance_to(goal)) / initial_dist_threhold
        #    print(factor)

            # reduce Vp the closer the initial position is to the goal
            Vp *= factor

        # Make vectors for position and goal
        position_v = np.array([position.x, position.y])
        goal_v = np.array([goal.x, goal.y])

        # calculate K
        K = initial_position.distance_to(goal)

        # calculate preferred speed
        preferred_speed = Vp * np.min(np.array([
            1,
            (position.distance_to(goal) / K)
        ]))

        # reduce speed if starting position is close to it's target
        # clip minimum speed to 0.1 (Not originally part of the algorithm)
        preferred_speed = preferred_speed if preferred_speed > 0.05 else 0.05

        # calculate preferred velocity
        preferred_velocity = preferred_speed * (goal_v - position_v) / position.distance_to(goal)
        preferred_velocity = (preferred_velocity[0], preferred_velocity[1])

        return preferred_speed, preferred_velocity

