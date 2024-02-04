import rclpy
import sys
from geometry_msgs.msg import Twist

def move_turtlebot(robot_num: int):
    rclpy.init()
    node = rclpy.create_node('turtlebot_controller')

    # Create a publisher for the /cmd_vel topic
    publisher = node.create_publisher(Twist, '/tb0/cmd_vel', 10)

    # Create a Twist message
    twist_msg = Twist()

    # Set linear and angular speeds (adjust as needed)
    twist_msg.linear.x = 0.2  # linear speed in m/s
    twist_msg.angular.z = 0.1  # angular speed in rad/s

    while rclpy.ok():
        # Publish the Twist message
        publisher.publish(twist_msg)
        node.get_logger().info('Moving TurtleBot')

        # Sleep for a short duration
        rclpy.spin_once(node)
        node.get_clock().sleep(0.1)

    # Stop the TurtleBot when the script is terminated
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 0.0
    publisher.publish(twist_msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    #robot_num = sys.argv[1]
    move_turtlebot(0)

