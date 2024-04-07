import numpy as np
from typing import Tuple, List, Set

from robot import Point, Robot


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