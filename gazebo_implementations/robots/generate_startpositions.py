import math
import json

def generate_robots_grid(n, grid_width, grid_height):
    robots = []
    for i in range(n):
        x = (i % grid_width)# * (100.0 / grid_width)
        y = (i // grid_width)# * (100.0 / grid_height)
        orientation = math.radians((i % 4) * 90)  # Simulate 0, 90, 180, 270 degrees
        robots.append({
            "name": f"robot{i+1}",
            "x": x,
            "y": y,
            "orientation": orientation
        })
    return robots

if __name__ == "__main__":
    grid_width = 4
    grid_height = 5

    for i in range(50):
        i += 1
        robots = generate_robots_grid(i, grid_width, grid_height)

        filename = f"{i}robots.json"
        with open(filename, "w") as f:
            json.dump(robots, f)
