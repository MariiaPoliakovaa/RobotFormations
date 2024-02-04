import rclpy
import sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class TurtlebotController:
    def __init__(self, robot_num):
        rclpy.init()
        self.node = rclpy.create_node("turtlebot_controller_{}".format(robot_num))

        # Create a publisher for the /cmd_vel topic
        self.publisher = self.node.create_publisher(
            Twist,
            '/tb{}/cmd_vel'.format(robot_num),
            10
        )

        # Create a subscriber for the /odom topic
        self.subscriber = self.node.create_subscription(
            Odometry,
            '/tb{}/odom'.format(robot_num),
            self.odom_callback,
            10
        )

        # Initialize the Twist message for controlling velocity
        self.twist_msg = Twist()

        # Initialize target 
        self.target_x = None
        self.target_y = None
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

    def run(self, x, y):
        # set goal
        self.target_x = x  # Set desired x-coordinate
        self.target_y = y  # Set desired y-coordinate

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
    robot_num = int(sys.argv[1])
    goal_x = int(sys.argv[2])
    goal_y = int(sys.argv[3])

    print("Controlling turtlebot No. {}".format(robot_num))
    print("Target position: ({}/{})".format(goal_x, goal_y))

    controller = TurtlebotController(robot_num)
    controller.run(goal_x, goal_y)

