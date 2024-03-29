from shapely.geometry import Polygon, Point
import shapely
import numpy as np
import matplotlib.pyplot as plt
import random
from scipy.spatial import Voronoi, voronoi_plot_2d

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
    # Create giant rectangle
    #rectangle_lower_left = (-50, -50)
    #rectangle_upper_right = (50, 50)
    #giant_rectangle = create_rectangle(rectangle_lower_left, rectangle_upper_right)
    buffer = shape.buffer(2)
    
    # generate points on the boundary of a giant rectangle around the polygon
    boundary_points = points_on_polygon_boundary(buffer, 1000)

    # get all points in one array
    all_points = np.vstack((boundary_points, points))

    # compute voronoi
    voronoi = Voronoi(all_points)

    # Extract Voronoi cells as Shapely polygons
    cells = []
    for region in voronoi.regions:
        if len(region) > 1 and -1 not in region:
            vertices = [voronoi.vertices[i] for i in region]
    
            contained = False
            for vertice in vertices:
                if polygon.contains(Point(vertice[0], vertice[1])):
                    contained = True
            if contained:
                cells.append(polygon.intersection(Polygon(vertices)))
    return cells


def compare_points(old_points, new_points):
    max_delta = 0.001

    for i in range(len(old_points)):
        delta = np.sqrt((old_points[i,0] - new_points[i,0])**2 + (old_points[i,1] - new_points[i,1])**2)
        if delta > max_delta :
            return False
    return True

def goal_generation(polygon, n):
    # generate random points in polygon
    points = generate_points(n, polygon)
    points = np.array([[point.x, point.y] for point in points])
    old_centroids = None

    i = True
    while i:
        voronoi_polygons = cell_polygons(points, polygon)
        centroids = [shapely.centroid(cell) for cell in voronoi_polygons]
        points = np.array([[point.x, point.y] for point in centroids])

        if old_centroids is not None:
            if compare_points(points, old_centroids):
                i = False

        if i%10 == 0:
            print("Iteration: ", i)
        
        old_centroids = points
    return points