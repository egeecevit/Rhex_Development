import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from transforms3d.quaternions import quat2mat

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
        fy = fx

        # Principal points (assuming the camera center is the image center)
        cx = width / 2
        cy = height / 2

        self.camera_matrix = np.array([[fx, 0, cx],
                                       [0, fy, cy],
                                       [0, 0, 1]])

        # Camera offset in the robot's coordinate frame
        self.camera_offset = np.array([[0.2695], [0.0055], [0.112]])  

        self.robot_pose = None
        self.static_point_world_frame = None
        self.fov_horizontal = 60  # in degrees
        self.fov_vertical = 45  # in degrees
        self.get_logger().info(f'Mark point node has started with camera matrix: \n{self.camera_matrix}')

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
        if self.static_point_world_frame is None:
            point_3d_robot = np.array([[1.0], [0.0], [0.0]])  # Only x, y, z
            self.static_point_world_frame = point_3d_robot + np.array([[self.robot_pose.position.x], 
                                                                        [self.robot_pose.position.y], 
                                                                        [self.robot_pose.position.z]]) + self.camera_offset
            self.get_logger().info(f'Static point in world frame: {self.static_point_world_frame}\n')

        self.get_logger().info(f'Received robot pose: {self.robot_pose.position.x}, {self.robot_pose.position.y}, {self.robot_pose.position.z}\n')

    def image_callback(self, msg):
        if self.robot_pose is None or self.static_point_world_frame is None:
            self.get_logger().info('Robot pose or static point not yet received\n')
            return

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")


        camera_position_in_world_frame = np.array([[self.robot_pose.position.x],
                                                [self.robot_pose.position.y], 
                                                [self.robot_pose.position.z]]) + self.camera_offset
        
        print(f'Camera position in world frame: {camera_position_in_world_frame}\n')

        # Update the transformation matrix with dynamic translation values
        transform_matrix_world_to_camera = np.array([[0.0, 1.0, 0.0, -camera_position_in_world_frame[1,0]],
                                                    [0.0, 0.0, -1.0, -camera_position_in_world_frame[2,0]],
                                                    [-1.0, 0.0, 0.0, -camera_position_in_world_frame[0,0]],
                                                    [0.0, 0.0, 0.0, 1.0]])
        
        print(f'Static point in world frame: {self.static_point_world_frame}\n')

        point_in_world_frame_homogeneous = np.append(self.static_point_world_frame, [[1.0]], axis=0)

        print(f'Point in world frame(homogeneous): {point_in_world_frame_homogeneous}\n')
                                                     
        point_in_camera_frame = transform_matrix_world_to_camera @ point_in_world_frame_homogeneous

        self.get_logger().info(f'Point in camera frame(homogeneous): {point_in_camera_frame}\n')


        # Project the 3D point to 2D using the camera intrinsic matrix
        point_2d_homogeneous = self.camera_matrix @ point_in_camera_frame[:3]
        self.get_logger().info(f'Point 2D homogeneous before normalization: {point_2d_homogeneous}\n')

        # Normalize the 2D coordinates by dividing by the third coordinate
        point_2d = point_2d_homogeneous[:2] / point_2d_homogeneous[2]
        self.get_logger().info(f'Projected 2D point: {point_2d}\n')

        # # Check if the point is within the image boundaries
        if 0 <= point_2d[0] and point_2d[0] < cv_image.shape[1] and 0 <= point_2d[1] and point_2d[1] < cv_image.shape[0]:
            # Draw the point on the image
            cv2.circle(cv_image, (int(point_2d[0]), int(point_2d[1])), 5, (0, 0, 255), -1)
            self.get_logger().info('Point drawn on image\n')
        else:
            self.get_logger().info('Projected point is outside the image boundaries\n')


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
