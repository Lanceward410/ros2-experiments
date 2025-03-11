import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class NavigateToGoal(Node):
    def __init__(self):
        super().__init__('navigate_to_goal')

        self.get_logger().info("🚀 Navigation Node Initialized!")

        # Create Action Client for `bt_navigator`
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.get_logger().info("⏳ Waiting for navigate_to_pose action server...")
        self._nav_client.wait_for_server()

        self.get_logger().info("✅ Action server is available!")

        # Send the goal
        self.send_goal()

    def send_goal(self):
        """ Sends a goal to the NavigateToPose action server """
        self.get_logger().info("📤 Sending goal to NavigateToPose...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()

        # Set the header
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp.sec = 1741654121
        goal_msg.pose.header.stamp.nanosec = 25101536

        # Set the goal pose (your provided position and orientation)
        goal_msg.pose.pose.position.x = 8.448183059692383
        goal_msg.pose.pose.position.y = -2.3569514751434326
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = -0.9986936886053002
        goal_msg.pose.pose.orientation.w = 0.05109712653310227

        self.get_logger().info(f"📌 Goal prepared: \n{goal_msg}")

        future = self._nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """ Handles response after sending goal """
        self.get_logger().info("📨 Received goal response from action server...")

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ NavigateToPose goal was rejected!")
            rclpy.shutdown()
            return

        self.get_logger().info("✅ Goal ACCEPTED! Waiting for the robot to reach the goal...")

        goal_handle.get_result_async().add_done_callback(self.navigation_complete_callback)

    def feedback_callback(self, feedback_msg):
        """ Handles feedback from the action server """
        self.get_logger().info(f"🔄 Receiving feedback: {feedback_msg}")

    def navigation_complete_callback(self, future):
        """ Final result after robot completes movement """
        self.get_logger().info("✅ Navigation Complete!")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToGoal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

