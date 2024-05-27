#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class Smartphone(Node):
    def __init__(self):
        super().__init__("smartphone")
        self.subscriber_ = self.create_subscription(String, "robot_news", self.callback_robot_news, 10) #Create a subscriber that subscribes to the topic "robot_news" 
        #The topic name should be same with the publisher
        #Callback function is needed because subscriber needs a function to call when a message is received
        self.get_logger().info("Smartphone has been started.")

    def callback_robot_news(self, msg): #Callback function takes a message as input. Callback function is used to process the message
        self.get_logger().info(msg.data) #Print the message data

def main(args=None):
    rclpy.init(args=args)
    node = Smartphone()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()