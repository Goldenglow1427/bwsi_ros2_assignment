#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose

class PoseSubscriberNode(Node):

    def __init__(self):
        super().__init__("pose_subscriber")

        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

    def pose_callback(self, msg: Pose):
        self.get_logger().info(f"({msg.x}, {msg.y})")


def main(args=None):
    rclpy.init(args=args)

    # Create the subscriber node.
    node = PoseSubscriberNode()
    
    # Keeps the node to keep active.
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
