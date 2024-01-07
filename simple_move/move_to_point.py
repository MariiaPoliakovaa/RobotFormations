import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class TurtlebotController:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('turtlebot_controller')

        # Create a publisher for the /cmd_vel topic
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

        # Create a subscriber for the /odom topic
        self.subscriber = self.node.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Initialize the Twist message for controlling velocity
        self.twist_msg = Twist()

        # Set the target position
        self.target_x = 2.0  # Set your desired x-coordinate
        self.target_y = 2.0  # Set your desired y-coordinate
        self.distance_threshold = 0.1  # Set a distance threshold for stopping the TurtleBot

    def odom_callback(self, msg):
        # Callback function to check the current position and move the TurtleBot
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Calculate the distance to the target position
        distance_to_target = math.sqrt((current_x - self.target_x)**2 + (current_y - self.target_y)**2)

        # Move the TurtleBot towards the target position
        if distance_to_target > self.distance_threshold:
            # Calculate the angle to the target position
            angle_to_target = math.atan2(self.target_y - current_y, self.target_x - current_x)

            # Set the linear and angular velocities
            self.twist_msg.linear.x = 0.2  # Set a linear speed (adjust as needed)
            self.twist_msg.angular.z = 0.5 * (angle_to_target - msg.pose.pose.orientation.z)  # Set an angular speed

            # Publish the Twist message
            self.publisher.publish(self.twist_msg)
        else:
            # Stop the TurtleBot when it reaches the target position
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            self.publisher.publish(self.twist_msg)

    def run(self):
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass

        # Stop the TurtleBot when the script is terminated
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.publisher.publish(self.twist_msg)

        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    controller = TurtlebotController()
    controller.run()

