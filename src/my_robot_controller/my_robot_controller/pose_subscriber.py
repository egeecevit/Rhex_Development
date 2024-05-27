#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__("pose_subscriber")#Node name
        self.pose_subscriber_= self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10) #Create a subscriber to the topic /turtle1/pose

    def pose_callback(self, msg: Pose):
        self.get_logger().info("(" + str(msg.x) + ", " + str(msg.y) + ")")


def main(args=None):
    rclpy.init(args=args) #Initialize the ROS2
    node = PoseSubscriberNode() #Create an object of the class PoseSubscriber
    rclpy.spin(node) #Keep the node running
    rclpy.shutdown() #Shutdown the ROS2