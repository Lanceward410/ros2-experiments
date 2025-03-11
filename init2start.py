import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # Create publisher
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Delay to ensure ROS 2 initializes before publishing
        time.sleep(1)

        # Publish the initial pose
        self.publish_initial_pose()

    def publish_initial_pose(self):
        """Publishes a PoseWithCovarianceStamped message to /initialpose"""
        msg = PoseWithCovarianceStamped()

        # Fill header
        msg.header.stamp.sec = 1741651688  # Use actual ROS time if needed
        msg.header.stamp.nanosec = 19358860
        msg.header.frame_id = "map"

        # Fill pose
        msg.pose.pose.position.x = 4.6769490242004395
        msg.pose.pose.position.y = -2.614407777786255
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = -0.23396250455722228
        msg.pose.pose.orientation.w = 0.9722456204382264

        # Fill covariance matrix
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
        ]

        # Publish message
        self.publisher.publish(msg)
        self.get_logger().info("📌 Initial Pose Published!")

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()

    try:
        rclpy.spin_once(node)  # Just publish once
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

