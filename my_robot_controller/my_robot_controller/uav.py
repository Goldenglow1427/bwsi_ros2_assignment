#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class UAV_Node(Node):
    
    def __init__(self):
        super().__init__("uav_awesome")

        # Get the logger and print the message.
        self.get_logger().info("UAV is Awesome")


def main(args=None):
    rclpy.init(args=args)

    node = UAV_Node()

    # Not necessary but personally prefer to do this.
    # rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()