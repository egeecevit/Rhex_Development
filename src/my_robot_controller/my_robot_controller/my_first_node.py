#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node") #Node name
        #self.get_logger().info("ROS2 RULES!") #Message to be printed in the terminal
        self.create_timer(1.0, self.timer_callback) #Create a timer with 1 second interval to call the timer_callback function
        self.counter_ = 0 #Initialize a counter

    def timer_callback(self): #Function to be called by the timer
        self.get_logger().info("Hello " + str(self.counter_)) #Print "Hello" to the terminal
        self.counter_ += 1 #Increment the counter
    


def main(args=None):
    rclpy.init(args=args) #Initialize the ROS2
    node = MyNode() #Create an object of the class MyNode
    rclpy.spin(node) #Keep the node running
    rclpy.shutdown() #Shutdown the ROS2

if __name__ == '__main__': # To be able to run the script using ROS2
    main()

