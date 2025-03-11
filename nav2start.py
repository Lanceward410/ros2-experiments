import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose, FollowPath
from geometry_msgs.msg import PoseStamped
import time

class NavigateToGoal(Node):
    def __init__(self):
        super().__init__('navigate_to_goal')

        self.get_logger().info("🚀 Navigation Node Initialized!")

        # Create Action Clients
        self._planner_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self._controller_client = ActionClient(self, FollowPath, '/follow_path')

        self.get_logger().info("⏳ Waiting for compute_path_to_pose action server...")
        self._planner_client.wait_for_server()

        self.get_logger().info("⏳ Waiting for follow_path action server...")
        self._controller_client.wait_for_server()

        self.get_logger().info("✅ Both action servers are available!")

        # Compute path first
        self.send_compute_path_goal()

    def send_compute_path_goal(self):
        """ Sends a goal to ComputePathToPose action server """
        self.get_logger().info("📤 Sending goal to ComputePathToPose...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = 4.6769490242004395
        goal_msg.pose.pose.position.y = -2.614407777786255
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = -0.23396250455722228
        goal_msg.pose.pose.orientation.w = 0.9722456204382264

        goal_msg.planner_id = "GridBased"

        self.get_logger().info(f"📌 Goal prepared: \n{goal_msg}")

        future = self._planner_client.send_goal_async(goal_msg)
        future.add_done_callback(self.compute_path_result_callback)

    def compute_path_result_callback(self, future):
        """ Handles the final result of ComputePathToPose """
        self.get_logger().info("📥 Received result from ComputePathToPose.")

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ ComputePathToPose goal was rejected!")
            rclpy.shutdown()
            return

        goal_handle.get_result_async().add_done_callback(self.follow_path)

    def follow_path(self, future):
        """ Sends the computed path to FollowPath to make the robot move """
        result = future.result().result
        num_waypoints = len(result.path.poses)

        if num_waypoints == 0:
            self.get_logger().error("❌ Path is empty! Robot cannot move.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"🚀 Path computed successfully! Path has {num_waypoints} waypoints.")
        self.get_logger().info("📤 Sending path to FollowPath...")

        follow_path_msg = FollowPath.Goal()
        follow_path_msg.path = result.path

        future = self._controller_client.send_goal_async(follow_path_msg)
        future.add_done_callback(self.follow_path_result_callback)

    def follow_path_result_callback(self, future):
        """ Handles the result of FollowPath """
        self.get_logger().info("📥 Received result from FollowPath.")

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ FollowPath goal was rejected!")
            rclpy.shutdown()
            return

        goal_handle.get_result_async().add_done_callback(self.navigation_complete_callback)

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

