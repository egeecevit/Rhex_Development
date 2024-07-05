#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


class ObtainImage(Node):
    def __init__(self):
        super().__init__("obtain_image")
        self.subscriber_ = self.create_subscription(
            Image,
            "/depth_camera/image_raw",
            self.image_callback,
            10
        )
        self.get_logger().info("Obtain Image node has been started.")
        self.bridge = CvBridge()

    def image_callback(self,msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            cv2.imshow("Camera Image", image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ObtainImage()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()


    