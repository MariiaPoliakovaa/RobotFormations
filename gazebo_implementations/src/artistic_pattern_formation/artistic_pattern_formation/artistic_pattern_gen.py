from shapely.geometry import Polygon, Point, MultiPoint
from shapely.ops import voronoi_diagram
from scipy.spatial import Voronoi
import shapely
import numpy as np
import matplotlib.pyplot as plt
import random
import ipdb

def euclidean_dist(a, b):
    return np.sqrt( (a[0] - b[0])**2 + (a[1] + b[1])**2 )

# Function to create a rectangle polygon
def create_rectangle(lower_left, upper_right):
    return Polygon([lower_left, (lower_left[0], upper_right[1]), upper_right, (upper_right[0], lower_left[1])])

# Function to create a circle polygon
def create_circle(center, radius, num_points=100):
    angles = np.linspace(0, 2*np.pi, num_points)
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    return Polygon(np.column_stack((x, y)))

# Function to create a triangle polygon
def create_triangle(vertices):
    return Polygon(vertices)

def generate_points(n: int, shape: Polygon):
    points = []
    i = 0

    # get minimum and maximum x and y values of polygon
    min_x, min_y, max_x, max_y = shape.bounds

    while i < n:
        x = random.uniform(min_x, max_x)
        y = random.uniform(min_y, max_y)
        point = Point(x, y)

        if shape.contains(point):
            points.append(point)
            i += 1
    return points

"""
def generate_points(num_points, polygon):
    points = []
    min_x, min_y, max_x, max_y = polygon.bounds
    while len(points) < num_points:
        random_point = Point(np.random.uniform(min_x, max_x), np.random.uniform(min_y, max_y))
        if polygon.contains(random_point):
            points.append(Point(random_point.x, random_point.y))
    return points
"""


def points_on_polygon_boundary(polygon, n):
    # Ensure input is a Polygon
    if not isinstance(polygon, Polygon):
        raise ValueError("Input must be a Shapely Polygon")

    # Get the linear ring (boundary) of the polygon
    boundary = polygon.boundary

    # Compute the length of the linear ring
    ring_length = boundary.length

    # Sample points along the linear ring
    step = ring_length / n
    sampled_points = []
    distance = 0.0
    while len(sampled_points) < n:
        point = boundary.interpolate(distance)
        sampled_points.append(np.array([point.x, point.y]))
        distance += step

    return sampled_points

def cell_polygons(points, shape):
    # Create buffer around shape
    buffer = shape.buffer(2)

    # create a boundary
    boundary = shape.buffer(10)

    # generate points on the boundary of a giant rectangle around the polygon
    buffer_points = points_on_polygon_boundary(buffer, 1000)

    # get all points in one array
    all_points = np.vstack((buffer_points, points))

    # compute voronoi
    voronoi = Voronoi(all_points)

    # Extract Voronoi cells as Shapely polygons
    cells = []
    for region in voronoi.regions:
        if len(region) > 1 and -1 not in region:
            vertices = [voronoi.vertices[i] for i in region]

            contained = True
            for vertice in vertices:
                if not boundary.contains(Point(vertice[0], vertice[1])):
                    contained = False
            if contained:
                cells.append(shape.intersection(Polygon(vertices)))
    return cells

def compare_points(old_points, new_points):
    max_delta = 0.001

    for i in range(len(old_points)):
        delta = np.sqrt((old_points[i,0] - new_points[i,0])**2 + (old_points[i,1] - new_points[i,1])**2)
        if delta > max_delta :
            return False
    return True

def goal_generation(polygon, n):
    # This is very hacky, but I dont know
    # how to fix the voronoi partition within a polygon.
    # Whenever there are less voronoi cells than there should be,
    # restart the computation.
    # TODO: Fix this shit
    error = True
    while error:
        error = False

        # generate random points in polygon
        points = generate_points(n, polygon)
        points = np.array([[point.x, point.y] for point in points])
        old_centroids = None

        run = True
        i = 0
        while run:
            voronoi_polygons = cell_polygons(points, polygon)
            centroids = [shapely.centroid(cell) for cell in voronoi_polygons]

            points = np.array([[point.x, point.y] for point in centroids])


            if len(points) != n:
                print("Encountered error, restarting computation!")
                error = True
                break

            if old_centroids is not None:
                if compare_points(points, old_centroids):
                    run = False

            old_centroids = points
            i += 1
    return points


if __name__ == '__main__':
    # Create a triangle
    triangle_vertices = [(1.5, -2.5), (4.5, -2.5), (3, 1.5)]
    triangle_polygon = create_triangle(triangle_vertices)

    goals = goal_generation(triangle_polygon, 5)
