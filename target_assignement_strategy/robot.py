import random
import numpy as np

from typing import List, Set, Tuple

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


class Robot:
    def __init__(self, x: float, y: float):
        self.position = Point(x, y)
        self.priority = None
        self.target = None

    def set_target(self, target: Point):
        self.target = target

    def set_priority(self, priority: int):
        self.priority = priority


def random_robots(n: int, x_min, x_max, y_min, y_max) -> List[Robot]:
    """
    Returns a list of Robots with random positions. The
    positions of the robots are constrained within a
    rectangle

    :param n     : Number of robots
    :param x_min : minimum x position of robots
    :param x_max : maximum x position of robots
    :param y_min : minimum y position of robots
    :param y_max : maximum y position of robots

    :returns     : List of randomly positioned robots
    """
    robots = []

    for i in range(n):
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        robots.append( Robot(x,y) )

    return robots


def circle_positions(n: int, radius: float) -> List[Point]:
    """
    Generates evenly spaced points on a circle

    :param n      : number of points
    :param radius : radius of the circle

    :returns      : List of points on a circle
    """
    points = []

    for theta in np.linspace(0, 2*np.pi, n+1):
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        points.append(Point(x, y))

    return points[:-1]


def greatest_distance_to_nearest_target(robots: Set[Robot], targets: Set[Point]) -> Tuple[Robot, Point]:
    """
    Returns the Robot whose distance to the nearest point is greater than for any other robot

    :param robots : Set of robots
    :param targets: Set of target points

    :returns      : Tuple of Robot and target point
    """

    # find the nearest target for each robot
    robots_nearest_targets = []
    
    for robot in robots:
        shortest_dist = np.inf
        for target in targets:
            robot_neares_target = None
            dist = robot.position.distance_to(target)
            if dist < shortest_dist:
                shortest_dist = dist
                robot_nearest_target = (robot, target)
        robots_nearest_targets.append(robot_nearest_target)

    # find the robot whose closest target is furthest away
    greatest_dist = -1
    robot_target = None
    
    for robot, target in robots_nearest_targets:
        dist = robot.position.distance_to(target)
        
        if dist > greatest_dist:
            greatest_dist = dist
            robot_target = (robot, target)

    return robot_target












