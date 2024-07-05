import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class GetOdom(Node):
    def __init__(self):
        super().__init__("get_odom")
        self.odom = Odometry()
        self.last_pose_x = None
        self.last_pose_y = None
        self.last_pose_theta = None
        self.point_pose_x = None
        self.point_pose_y = None
        self.point_pose_theta = None
        self.init_odom_state = False

        self.subscriber_ = self.create_subscription(
            Odometry,
            "/odom/robot_pos",
            self.odom_callback,
            10
        )
        self.get_logger().info("get_odom node has been started")

    def odom_callback(self, msg):
        if not self.init_odom_state:
            self.last_pose_x = msg.pose.pose.position.x
            self.last_pose_y = msg.pose.pose.position.y
            _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

            self.init_odom_state = True
            self.get_logger().info(f'Initial pose_x: {self.last_pose_x}, pose_y: {self.last_pose_y}, pose_theta: {self.last_pose_theta}')
            self.assign_point()  # Assign point once after initial odom is received
            
            # Unsubscribe from the topic after receiving the initial odom data
            self.destroy_subscription(self.subscriber_)

    def euler_from_quaternion(self, quat):
        """
        Convert quaternion (w in last place) to euler roll, pitch, yaw.
        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def assign_point(self):
        self.point_pose_x = self.last_pose_x + 1.0
        self.point_pose_y = self.last_pose_y
        self.point_pose_theta = self.last_pose_theta
        print(f"Initial Point Pose: {self.point_pose_x}, {self.point_pose_y}, {self.point_pose_theta}")

class ImageSubscriber(Node):
    def __init__(self, odom_node):
        super().__init__("image_subscriber")
        self.subscriber_ = self.create_subscription(
            Image,
            "/depth_camera/image_raw",
            self.image_callback,
            10
        )
        self.get_logger().info("Image Subscriber node has been started.")
        self.bridge = CvBridge()
        self.image = None
        self.odom_node = odom_node

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.visualize_point()

    def visualize_point(self):
        if not self.odom_node.init_odom_state or self.image is None:
            return
        x = self.odom_node.last_pose_x
        y = self.odom_node.last_pose_y
        theta = self.odom_node.last_pose_theta
        p_x = self.odom_node.point_pose_x
        p_y = self.odom_node.point_pose_y
        p_theta = self.odom_node.point_pose_theta

        point_tf_x = p_x - x
        point_tf_y = p_y - y

        p_x_world = x + (point_tf_x) * np.cos(theta) - (point_tf_y) * np.sin(theta)
        p_y_world = y + (point_tf_x) * np.sin(theta) + (point_tf_y) * np.cos(theta)

        img_point_x = int(p_x_world * 100)  # Scale for visualization
        img_point_y = int(p_y_world * 100)

        cv2.circle(self.image, (img_point_x, img_point_y), 5, (0, 0, 255), -1)

        # Display the image
        cv2.imshow("Image with Point", self.image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node_odom = GetOdom()
    node_image = ImageSubscriber(node_odom)

    rclpy.spin_once(node_odom)  # Ensure the odom node gets its initial data

    rclpy.spin(node_image)

    node_odom.destroy_node()
    node_image.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
