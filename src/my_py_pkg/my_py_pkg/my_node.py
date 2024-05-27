#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self): #Constructor
        super().__init__("py_test") #Create a node same as node = Node("py_test")
        self.counter_ = 0
        self.get_logger().info("Hello ROS2") #Print a message
        self.create_timer(0.5, self.timer_callback) #Create a timer with 0.5s period

    def timer_callback(self): #Callback function
        self.counter_ += 1
        self.get_logger().info("Hello " + str(self.counter_))

def main(args=None):
    rclpy.init(args=args) #Start ROS communication
    node = MyNode() #Create a node
    rclpy.spin(node)#Keep the node running
    rclpy.shutdown() #Stop ROS communication



if __name__ == "__main__":
    main()