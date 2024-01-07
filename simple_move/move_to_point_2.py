import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes

class TurtlebotNavigator:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('turtlebot_navigator')

        # Create a client for the navigation action
        self.navigate_client = self.node.create_client(NavigateToPose, '/navigate_to_pose')

        # Create a service client for managing the lifecycle of the navigation nodes
        self.lifecycle_manager_client = self.node.create_client(ManageLifecycleNodes, '/lifecycle_manager/manage_nodes')

        # Wait for the action server to be available
        while not self.navigate_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Navigation action server not available. Waiting...')

        # Start the navigation nodes
        self.start_navigation_nodes()

    def start_navigation_nodes(self):
        # Start the navigation nodes
        lifecycle_manager_request = ManageLifecycleNodes.Request()
        lifecycle_manager_request.action = ManageLifecycleNodes.Request.STARTUP
        self.node.get_logger().info('Starting navigation nodes...')
        self.lifecycle_manager_client.call(lifecycle_manager_request)

        # Wait for the nodes to start
        rclpy.spin_until_future_complete(self.node, self.lifecycle_manager_client.response_future)

    def send_goal(self, target_x, target_y):
        # Create a goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = target_x
        goal_pose.pose.position.y = target_y
        goal_pose.pose.orientation.w = 1.0

        # Create a NavigateToPose action goal
        action_goal = NavigateToPose.Goal()
        action_goal.pose = goal_pose.pose

        # Send the goal to the navigation action server
        self.node.get_logger().info('Sending goal to navigation server...')
        self.navigate_client.wait_for_service()
        future = self.navigate_client.send_goal_async(action_goal)

        # Wait for the navigation action to complete
        rclpy.spin_until_future_complete(self.node, future)

    def stop_navigation_nodes(self):
        # Shutdown the navigation nodes
        lifecycle_manager_request = ManageLifecycleNodes.Request()
        lifecycle_manager_request.action = ManageLifecycleNodes.Request.SHUTDOWN
        self.node.get_logger().info('Shutting down navigation nodes...')
        self.lifecycle_manager_client.call(lifecycle_manager_request)

        # Wait for the nodes to shutdown
        rclpy.spin_until_future_complete(self.node, self.lifecycle_manager_client.response_future)

    def run(self, target_x, target_y):
        try:
            self.send_goal(target_x, target_y)
        except Exception as e:
            self.node.get_logger().error(f'Error: {e}')
        finally:
            self.stop_navigation_nodes()
            self.node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    navigator = TurtlebotNavigator()
    navigator.run(2.0, 2.0)  # Set your desired target coordinates

