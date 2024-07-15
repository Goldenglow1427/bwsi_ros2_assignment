#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from turtlesim.srv import SetPen

from functools import partial

class TurtleControllerNode(Node):

    color: int # The current color, -1 means red, 0 means default, and 1 means green

    def __init__(self):
        super().__init__("turtle_controller")

        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.get_logger().info("Turtle controller has been started and properly initialized.")

        self.color = 0

    def pose_callback(self, pose: Pose):
        cmd = Twist()

        if pose.x > 9 or pose.x < 2 or pose.y > 9 or pose.y < 2:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.95
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0

        if pose.x > 5.5 and self.color != -1:
            self.get_logger().info("Set color to red")
            self.call_set_pen_service(255, 0, 0, 4, 0)
            self.color = -1
        elif pose.x <= 5.5 and self.color != 1:
            self.get_logger().info("Set color to green")
            self.call_set_pen_service(0, 255, 0, 4, 0)
            self.color = 1

        self.cmd_vel_publisher_.publish(cmd)
        
    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, "/turtle1/set_pen")

        # Try to connect to the service.
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service ...")

        self.get_logger().info("The service is up!")

        request = SetPen.Request()
        request.r, request.g, request.b, request.width, request.off = r, g, b, width, off
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.set_pen_callback))

    def set_pen_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()