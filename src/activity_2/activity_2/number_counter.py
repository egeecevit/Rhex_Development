#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberCounter(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.counter_ = 0
        #Since publisher is used in the callback function we need to create the publisher first
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        #Normally publishers require a timer to publish messages periodically. 
        #But in this case, we will publish a message when a message is received using callback_subscriber
        self.subscriber_ = self.create_subscription(Int64, "number", self.callback_subscriber, 10)
        self.get_logger().info("Number Counter has been started.")

    def callback_subscriber(self, msg):
        self.counter_ += msg.data
        new_msg = Int64()
        new_msg.data = self.counter_ #Set the data of the new message
        self.publisher_.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NumberCounter()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    main()