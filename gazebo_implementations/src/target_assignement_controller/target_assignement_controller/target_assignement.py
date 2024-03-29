import numpy as np
from typing import Tuple, List, Set
from copy import copy, deepcopy

from .turtlebot import Turtlebot, Point

def greatest_distance_to_nearest_target(robots: Set[Turtlebot], targets: Set[Point]) -> Tuple[Turtlebot, Point]:
    """
    Returns the Turtlebot whose distance to the nearest point is greater than for any other robot

    :param robots : Set of robots
    :param targets: Set of target points

    :returns      : Tuple of Turtlebot and target point
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



def target_assignement_strategy(robots: List[Turtlebot], targets: List[list], prio:str="increasing") -> Set[Turtlebot]:
    """
    Assignes targets and priorities to robots.
    Uses either the first or second algorithm. The first algorithm initializes the priority as one and increases it
    with every matched robot. The second algorithm initializes the priority as n and decreases it with every matched robot.

    :param robots : List of all robots to be assigned a target
    :param targets: List of all targets
    :param prio   : "decreasing" or "increasing", chooses which algorithm to use

    :returns      : Set of robots with assigned targets
    """
    targets = [Point(target[0], target[1]) for target in targets]

    # check if number of robots equals number of targets
    if len(robots) != len(targets):
        raise ValueError("Number of robots must match number of targets")

    # initial assignement
    RT = set()
    RN = set()
    TN = set()
    TR = set()

    RN.update(deepcopy(robots))
    TN.update(deepcopy(targets))

    # initial priority
    if prio == "increasing":
        p = 1
    elif prio == "decreasing":
        p = len(robots)
    else:
        raise ValueError("prio must be either 'increasing' or 'decreasing'")

    # do target assignement
    while len(TN) != 0:
        # find robot whose distance to the nearest target is greater than for any other robot
        robot, target = greatest_distance_to_nearest_target(RN, TN)

        # set robot target
        robot.set_target(target)

        # move robot from RN to RT
        RN.remove(robot)
        RT.add(robot)

        # move target from TN to TR
        TN.remove(target)
        TR.add(target)

        # assign priority to robot
        robot.set_priority(p)

        # update priority
        if prio == "increasing":
            p += 1
        else:
            p -= 1

    # return the robots with their targets assigned
    return RT
