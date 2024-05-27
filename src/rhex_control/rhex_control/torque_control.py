#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class TorqueControl(Node):
    def __init__(self):
        super().__init__("torque_control")
        
        self.publisher_ = self.create_publisher(Float64MultiArray,"/effort_controller/commands", 10)

        self.start_time_ = time.time()

        self.timer_ = self.create_timer(0.1, self.publish_torque)

        self.get_logger().info("Torque Control has been started")


    def publish_torque(self):
        msg = Float64MultiArray()
        current_time = time.time() - self.start_time_
        msg.data = [
            4 * np.sin(current_time),
            3 * np.sin(current_time),
            4 * np.cos(current_time),
            3 * np.cos(current_time),
            6 * np.sin(current_time),
            6 * np.cos(current_time),
        ]
        self.publisher_.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = TorqueControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()