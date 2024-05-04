from rclpy.node import Node

class BaseController(Node):
    def __init__(self, node_name, robot_names, patterns):
        super().__init__(node_name)

        # Init a dictionary to log robot positions
        self.positions = {
            name: [] for name in robot_names
        }
        self.positions['timestamp'] = []

        # Simulation time
        self.simulation_time = None

        # Turtlebot parameters
        self.max_angular_speed = 2.84
        self.max_linear_speed = 0.22
        self.max_delta_velocity = 0.6
        self.max_delta_omega = np.pi/2

        # make an endless cycling iterator for the patterns
        self.pattern_names = cycle(patterns.keys())
        self.goal_positions = patterns


    def log_positions(self):
        self.positions['timestamp'] = self.simulation_time
        for robot in self.robots.values():
            self.positions[robot.name].append(
                [
                    robot.position.x,
                    robot.position.y,
                    robot.w
                ]
            )

