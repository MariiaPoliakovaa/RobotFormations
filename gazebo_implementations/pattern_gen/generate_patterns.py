import sys
import json
import ipdb
import shapely
import numpy as np
from shapely.geometry import Polygon, Point
from artistic_pattern_gen import goal_generation, create_circle, create_triangle, create_rectangle, create_star, create_ring

def numpy_array_to_list(obj):
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, dict):
        return {key: numpy_array_to_list(value) for key, value in obj.items()}
    elif isinstance(obj, list):
        return [numpy_array_to_list(item) for item in obj]
    else:
        return obj

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
    rectangle_lower_left = (-1.2, -1.2)
    rectangle_upper_right = (1.2, 1.2)
    rectangle_polygon = create_rectangle(rectangle_lower_left, rectangle_upper_right)

    # Create a star
    star_polygon = create_star((0,0), 2, 1, 5)

    # create a ring
    ring_polygon = create_ring(circle_center, 1.8, 1)



    patterns = {
        'circle': circle_polygon,
        'triangle': triangle_polygon,
        'ring': ring_polygon,
        'rectangle': rectangle_polygon
        #'star': star_polygon
    }

    # create goal positions from patterns
    goal_patterns = {}
    for name, pattern in patterns.items():
        goal_positions = goal_generation(pattern, n)
        goal_patterns[name] = goal_positions

    # format for json
    goal_patterns = numpy_array_to_list(goal_patterns)

    # write patterns to json
    with open(f"patterns/{n}robots_patterns.json", "w") as f:
        json.dump(goal_patterns, f, indent=4)


if __name__ == '__main__':
    for num_robots in range(40, 41):
        main(num_robots)

