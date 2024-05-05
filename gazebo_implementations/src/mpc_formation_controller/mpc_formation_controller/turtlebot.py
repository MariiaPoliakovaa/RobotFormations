import numpy as np
import casadi as cas

class MPCTurtleBot:
    def __init__(self,
                 name: str,
                 max_angular_speed: float,
                 max_linear_speed: float):
        self.name = name
        self.max_angular_speed = max_angular_speed
        self.max_linear_speed = max_linear_speed
        self.target_radius = 0.2
        self.target = None

        self.state = None

        self.position = None
        self.w = None

    def set_state(self, x: float, y: float, w: float):
        self.state = [x, y, w]

    def stop_moving(self):
        pass

    def set_target(self, x: float, y: float, w: float):
        """
        Sets target position and orientation
        for the robot
        """
        self.target = [x, y, w]

    def reached_target(self) -> bool:
        """
        Returns True if the robot has reached
        it's target, else False
        """
        if self.state == None or self.target == None:
            return False

        pos = np.array(self.state[:2])
        target = np.array(self.target[:2])

        dist = np.linalg.norm(pos - target)

        if dist < self.target_radius:
            return True
        else:
            return False
