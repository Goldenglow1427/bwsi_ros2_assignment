#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    """
    My first ROS2 node.
    """
    def __init__(self):
        super().__init__("first_node")
        self.counter_ = 0
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f"Hello {self.counter_}")
        self.counter_ += 1


def main(args=None):
    rclpy.init(args=args) # Starts the ROS2 communication

    # Creates the node.
    node = MyNode()

    # Keep the node alive under it's killed.
    rclpy.spin(node)

    rclpy.shutdown() # Close the initialized content (ROS2).


if __name__ == "__main__":
    main()
