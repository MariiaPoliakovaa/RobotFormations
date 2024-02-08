def target_and_prio_assignement(self):
    """
    Assignes targets and priorities to controllers.
    Uses either the first or second algorithm. The first algorithm initializes the priority as one and increases it
    with every matched robot. The second algorithm initializes the priority as n and decreases it with every matched robot.

    :param robots : List of all robots to be assigned a target
    :param targets: List of all targets
    :param prio   : "decreasing" or "increasing", chooses which algorithm to use

    :returns      : Set of robots with assigned targets
    """
    controllers = self.turtlebot_controllers
    self.targets = self.target_positions

    # check if number of controllers equals number of targets
    if len(controllers) != len(targets):
        raise ValueError("Number of controllers must match number of targets")

    # initial assignement
    RT = set()
    RN = set()
    TN = set()
    TR = set()

    RN.update(deepcopy(controllers))
    TN.update(deepcopy(targets))

    # initial priority
    if prio == "increasing":
        p = 1
    elif prio == "decreasing":
        p = len(controllers)
    else:
        raise ValueError("prio must be either 'increasing' or 'decreasing'")

    # do target assignement
    while len(TN) != 0:
        # find robot whose distance to the nearest target is greater than for any other robot
        turtlebot_controller, target = self.greatest_distance_to_nearest_target(RN, TN)

        # set turtlebot_controller target
        turtlebot_controller.set_target(target)

        # move turtlebot_controller from RN to RT
        RN.remove(turtlebot_controller)
        RT.add(turtlebot_controller)

        # move target from TN to TR
        TN.remove(target)
        TR.add(target)

        # assign priority to turtlebot_controller
        turtlebot_controller.set_priority(p)

        # update priority
        if prio == "increasing":
            p += 1
        else:
            p -= 1

    self.turtlebot_controllers = RT

def greatest_distance_to_nearest_target(self, controllers, targets):
    """
    Returns the Robot whose distance to the nearest point is greater than for any other robot

    :param controllers : Set of controllers
    :param targets: Set of target points

    :returns      : Tuple of Robot and target point
    """

    # find the nearest target for each robot
    robots_nearest_targets = []

    for robot in controllers:
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
