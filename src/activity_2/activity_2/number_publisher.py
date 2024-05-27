#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberPublisher(Node):
    def __init__(self):
        super().__init__("number_publisher")
        #self.declare_parameter("test123") #Declare a parameter with the name "test123"
        self.declare_parameter("number_to_publish",2) #Declare a parameter with the name "number_to_publish"
        self.declare_parameter("publish_freq", 1.0)
        
        #previously self.number_ = 2 but we want it to be a parameter which can be changed in the command line
        self.number_ = self.get_parameter("number_to_publish").value #Get the value of the parameter "number_to_publish"
        self.publish_freq_ = self.get_parameter("publish_freq").value #Get the value of the parameter "publish_freq"
        
        self.publisher_ = self.create_publisher(Int64, "number", 10)
        self.timer_ = self.create_timer(1 / self.publish_freq_, self.publisher_msg)
        # 1 / self.publish_freq_ is because of value is integer and we want to get the hz value
        # self.timer_ = self.create_timer(1, self.publisher_msg) we want the publish freq to be configurable thus we use the parameter as above
        self.get_logger().info("Number Publisher has been started")


    def publisher_msg(self):
        msg = Int64()
        msg.data = self.number_
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    rclpy.shutdown()




if __name__ == '__main__':
    main()