#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String #Import the String message type for package example_interfaces
#since new package is used, add it to the package.xml

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station") #Create a node named "robot_news_station"

        self.robot_name_ = "C3PO"
        self.publisher_ = self.create_publisher(String, "robot_news", 10) #Create a publisher that publishes String messages to the topic "robot_news"
        self.timer_ = self.create_timer(0.5, self.publish_news) #Create a timer that calls the publish_news function every 0.5s
        self.get_logger().info("Robot News Station has been started") # Info flag to be printed in the terminal

    def publish_news(self): #Function to publish a message
        msg = String() #Create a new String message instance
        msg.data = "Hi this is, "+ str(self.robot_name_) + " from the robot news station." #Set the message data to "Hello" .data is gathered from ros2 interface shown in the terminal
        self.publisher_.publish(msg) #Publish the message using publisher_ method


def main(args=None):
    rclpy.init(args=args) #Initilize ROS communication
    node = RobotNewsStationNode() #Create a node instance
    rclpy.spin(node) #Keep the node running
    rclpy.shutdown() #Close the ROS communication



if __name__ == "__main__":
    main()