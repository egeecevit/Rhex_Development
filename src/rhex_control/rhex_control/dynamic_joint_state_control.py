#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from control_msgs.msg import InterfaceValue

class DynamicJointStateControl(Node):
    def __init__(self):
        super().__init__("dynamic_joint_state_control")

        self.publisher_ = self.create_publisher(DynamicJointState,"dynamic_joint_states", 10)

        self.get_logger().info("Dynamic Joint State Control has been started")


    def publish_torque(self):
        msg = DynamicJointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = {'rear_left_joint', 'rear_right_joint', 'mid_left_joint', 'mid_right_joint', 'front_left_joint', 'front_right_joint'}
        msg.interface_values = {
            InterfaceValue().interface_names: msg.joint_names,
            InterfaceValue().values: {4, 3, 4, 3, 6, 6}
        }

        self.publisher_.publish(msg)





def main(args=None):
    rclpy.init(args=args)
    node = DynamicJointStateControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()