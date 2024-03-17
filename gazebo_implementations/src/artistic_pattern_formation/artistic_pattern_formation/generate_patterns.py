import sys
import json
import ipdb
import shapely
import numpy as np
from shapely.geometry import Polygon, Point
from artistic_pattern_gen import goal_generation, create_circle, create_triangle, create_rectangle

def main(n:int):
    """
    creates a json file with patterns for robot
    formations.

    :param n: Number of robots to create pattern for
    """
    # Create a circle
    circle_center = (0, 0)
    circle_radius = 1.5
    circle_polygon = create_circle(circle_center, circle_radius)

    # Create a triangle
    triangle_vertices = [(-1.5, -1.5), (1.5, -1.5), (0, 1.5)]
    triangle_polygon = create_triangle(triangle_vertices)

    # Create a rectangle
    rectangle_lower_left = (-1.5, -1.5)
    rectangle_upper_right = (1.5, 1.5)
    rectangle_polygon = create_rectangle(rectangle_lower_left, rectangle_upper_right)


    patterns = [
        circle_polygon,
        triangle_polygon,
        rectangle_polygon
    ]

    # put pattern in all positive coordinates
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

