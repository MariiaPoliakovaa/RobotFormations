import sys
import json
import ipdb
import shapely
import numpy as np
from shapely.geometry import Polygon, Point
from artistic_pattern_gen import goal_generation, create_circle

def main(n:int):
    """
    creates a json file with patterns for robot
    formations.

    :param n: Number of robots to create pattern for
    """
    # create patterns (only one for now)
    circle = create_circle([0,0], 1.5)
    patterns = [circle]

    # put pattern in all positive coordinates
    ipdb.set_trace()
    pos_patterns = []
    for pattern in patterns:
        min_x, min_y, max_x, max_y = pattern.bounds
        pos_patterns.append(
            shapely.affinity.translate(pattern, xoff=-min_x, yoff=-min_y)
        )

    # create goal positions from patterns
    goal_patterns = []
    for pattern in pos_patterns:
        goal_positions = goal_generation(pattern, n)
        goal_patterns.append(goal_positions.tolist())


    # write patterns to json
    with open("../data/patterns.json", "w") as f:
        json.dump(goal_patterns, f)


if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Please provide the number of robots to create the pattern for!")
        exit

    num_robots = int(sys.argv[1])

    #ipdb.set_trace()
    main(num_robots)

