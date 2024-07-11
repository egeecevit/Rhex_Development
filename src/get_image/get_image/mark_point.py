import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('mark_point')
        self.subscription = self.create_subscription(
            Image,
            '/depth_camera/image_raw',
            self.image_callback,
            10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom/robot_pos',
            self.odom_callback,
            10)
        self.bridge = CvBridge()

        # Camera intrinsic parameters
        width = 640
        height = 480
        horizontal_fov = 1.047198  # radians

        # Calculate the focal lengths
        fx = width / (2 * np.tan(horizontal_fov / 2))
        fy = fx * (height / width)

        # Principal points (assuming the camera center is the image center)
        cx = width / 2
        cy = height / 2

        self.camera_matrix = np.array([[fx, 0, cx],
                                       [0, fy, cy],
                                       [0, 0, 1]])

        # Camera offset in the robot's coordinate frame
        self.camera_offset = np.array([[0.2695], [0.0055], [0.112]])  

        self.robot_pose = None
        self.fov_horizontal = 60  # in degrees
        self.fov_vertical = 40  # in degrees
        self.get_logger().info(f'Mark point node has started with camera matrix: \n{self.camera_matrix}')

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
        self.get_logger().info(f'Received robot pose: {self.robot_pose.position.x}, {self.robot_pose.position.y}, {self.robot_pose.position.z}')

    def is_point_in_fov(self, point_3d):
        x, y, z = point_3d
        horizontal_angle = np.degrees(np.arctan2(x, z))
        vertical_angle = np.degrees(np.arctan2(-y, z))
        self.get_logger().info(f'Horizontal angle: {horizontal_angle}, Vertical angle: {vertical_angle}')
        return (abs(horizontal_angle) <= self.fov_horizontal / 2) and (abs(vertical_angle) <= self.fov_vertical / 2)

    def image_callback(self, msg):
        if self.robot_pose is None:
            self.get_logger().info('Robot pose not yet received')
            return

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Get the robot orientation as a rotation matrix
        orientation_q = self.robot_pose.orientation
        orientation = R.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        rotation_matrix = orientation.as_matrix()

        # 3D point 1 meter ahead in the world frame
        point_3d_robot = np.array([[1.0], [0.0], [0.0]])  # Only x, y, z


        # Transform the point to the world frame using the robot's position
        point_in_world_frame = point_3d_robot + self.camera_offset + np.array([[self.robot_pose.position.x], [self.robot_pose.position.y], [self.robot_pose.position.z]])

        self.get_logger().info(f'Point in world frame: {point_in_world_frame.flatten()}')

        if point_in_world_frame[2, 0] <= 0:
            self.get_logger().info('Point is behind the camera')
            return

        point_3d = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]) @ rotation_matrix @ point_in_world_frame
        # Project the 3D point to 2D
        point_2d = self.camera_matrix @ np.vstack((point_3d, [1]))[:3, :]
        point_2d = point_2d / point_2d[2]  # Normalize by the third coordinate

        self.get_logger().info(f'Projected 2D point: {point_2d}')

        # if not self.is_point_in_fov(point_2d.flatten()):
        #     self.get_logger().info('Point is outside the FOV')
        #     return  # Point is outside the FOV

        # Check if the point is within the image boundaries
        if 0 <= point_2d[0, 0] < cv_image.shape[1] and 0 <= point_2d[1, 0] < cv_image.shape[0]:
            # Draw the point on the image
            cv2.circle(cv_image, (int(point_2d[0, 0]), int(point_2d[1, 0])), 5, (0, 0, 255), -1)
            self.get_logger().info('Point drawn on image')
        else:
            self.get_logger().info('Projected point is outside the image boundaries')

        # Display the image
        cv2.imshow('Image with 3D point', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
