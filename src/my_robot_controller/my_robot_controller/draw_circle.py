#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub_ = self.create_publisher(Twist,"turtle1/cmd_vel",10)
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Draw Circle Node has been started")

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_pub_.publish(msg)
        self.get_logger().info("Velocity command has been sent")     



def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
