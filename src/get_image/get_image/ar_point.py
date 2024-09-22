#! /usr/bin/env python3
import rclpy
import rclpy.duration
from rclpy.node import Node
import rclpy.timer
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import TransformStamped
import time
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('ar_point')
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
        self.pc_subscriber_ = self.create_subscription(
            PointCloud2,
            "/depth_camera/points",
            self.pc_callback,
            10
        )
        self.mark_publisher_ = self.create_publisher(
            PointStamped,
            '/rhex_mark',
            10
        )
        self.bridge = CvBridge()

        # Camera intrinsic parameters
        self.width = 640
        self.height = 480
        horizontal_fov = 1.047198  # radians

        # Calculate the focal lengths
        fx = self.width / (2 * np.tan(horizontal_fov / 2))
        fy = fx

        # Principal points (assuming the camera center is the image center)
        cx = self.width / 2
        cy = self.height / 2

        self.camera_matrix = np.array([[fx, 0, cx],
                                       [0, fy, cy],
                                       [0, 0, 1]])

        # Camera offset in the robot's coordinate frame
        self.camera_offset = np.array([-0.0075, 0.0105, 0.025])  

        self.robot_pose = None
        self.static_point_world_frame = None
        self.fov_horizontal = 60  # in degrees
        self.fov_vertical = 45  # in degrees
        self.window_name = 'Image with 3D point'
        self.clicked_point = None
        self.point_cloud = None
        self.point_x = 0.0
        self.point_y = 0.0
        self.point_z = 0.0
        self.point_in_world_frame = None

        cv2.namedWindow(self.window_name)
        self.odom_timestamp = None
        self.br = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)



    def pc_callback(self, msg: PointCloud2):
        self.point_cloud = pc2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=True, reshape_organized_cloud=True)
        
    def broadcast_transform(self):
        if self.robot_pose is not None and self.odom_timestamp is not None:
            t = TransformStamped()
            t.header.stamp = self.odom_timestamp
            t.header.frame_id = 'world'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.robot_pose.position.x
            t.transform.translation.y = self.robot_pose.position.y
            t.transform.translation.z = self.robot_pose.position.z

            q = self.robot_pose.orientation
            t.transform.rotation.x = q.x
            t.transform.rotation.y = q.y
            t.transform.rotation.z = q.z
            t.transform.rotation.w = q.w

            self.br.sendTransform(t)
        else:
            return

    def listen_transform(self, x, y, z):
        if self.tf_buffer.can_transform('world', 'camera_depth_frame', rclpy.time.Time(seconds=0)):
            try:
                transform = self.tf_buffer.lookup_transform('world', 'camera_depth_frame', rclpy.time.Time(seconds=0))
                point_stamped = PointStamped()
                point_stamped.header.frame_id = 'camera_depth_frame'
                point_stamped.header.stamp = self.odom_timestamp
                point_stamped.point.x = x
                point_stamped.point.y = y
                point_stamped.point.z = z
                point_in_world_frame_stamped = do_transform_point(point_stamped, transform)
                point_in_world_frame = np.array([point_in_world_frame_stamped.point.x,
                                                  point_in_world_frame_stamped.point.y,
                                                  point_in_world_frame_stamped.point.z])
                #self.get_logger().info(f'Point in camera frame: {point_in_camera_frame}\n')

            except TransformException as e:
                self.get_logger().warn(f'Failed to get transform: {e}')
                return None
        
            return point_in_world_frame
        else:
            return
        
    def listen_transform_reverse(self, x, y, z):
        if self.tf_buffer.can_transform('camera_depth_frame', 'world', rclpy.time.Time(seconds=0)):
            try:
                transform = self.tf_buffer.lookup_transform('camera_depth_frame', 'world', rclpy.time.Time(seconds=0))
                point_stamped = PointStamped()
                point_stamped.header.frame_id = 'world'
                point_stamped.header.stamp = self.odom_timestamp
                point_stamped.point.x = x
                point_stamped.point.y = y
                point_stamped.point.z = z
                point_in_camera_frame_stamped = do_transform_point(point_stamped, transform)
                point_in_camera_frame = np.array([point_in_camera_frame_stamped.point.x,
                                                  point_in_camera_frame_stamped.point.y,
                                                  point_in_camera_frame_stamped.point.z])
                #self.get_logger().info(f'Point in camera frame: {point_in_camera_frame}\n')

            except TransformException as e:
                self.get_logger().warn(f'Failed to get transform: {e}')
                return None
        
            return point_in_camera_frame
        else:
            return

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
        self.odom_timestamp = msg.header.stamp
        self.broadcast_transform()

    def mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_point = np.array([x, y, 1])
            self.process_click()

    def process_click(self):
        if self.clicked_point is None or self.point_cloud is None:
            return
        
        u, v, k = self.clicked_point
        idx = v * self.width + u  # Calculate the flat index for the point cloud

        # Check if the index is valid and within bounds
        if idx < 0 or idx >= self.point_cloud.shape[0]:
            self.get_logger().warn(f'Clicked index {idx} is out of bounds for point cloud size {self.point_cloud.shape[0]}')
            return

        # Get the 3D point corresponding to the clicked pixel
        point = self.point_cloud[idx]
        self.point_x, self.point_y, self.point_z = float(point[0]), float(point[1]), float(point[2])

        # Check if the point is valid
        if not np.isfinite([self.point_x, self.point_y, self.point_z]).all():
            self.get_logger().warn(f'Invalid point clicked at index {idx}: {point}')

        # Transform the 3D point to the world frame
        self.point_in_world_frame = self.listen_transform(self.point_x, self.point_y, self.point_z)

        # Publish the point to visualize it in RViz
        self.publish_point(self.point_in_world_frame[0], self.point_in_world_frame[1], self.point_in_world_frame[2])


    def publish_point(self, x, y, z):
        point = PointStamped()
        point.header.frame_id = 'world'
        point.header.stamp = self.odom_timestamp
        point.point.x = x
        point.point.y = y
        point.point.z = z

        self.mark_publisher_.publish(point)

    def image_callback(self, msg):
        if self.robot_pose is None:
            self.get_logger().info('Robot pose not yet received\n')
            return

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        cv2.setMouseCallback(self.window_name, self.mouse_click)

        if self.clicked_point is not None:
            point_camera_frame = self.listen_transform_reverse(self.point_in_world_frame[0], self.point_in_world_frame[1], self.point_in_world_frame[2])

            point_2d_homogeneous = self.camera_matrix @ point_camera_frame

            point_2d = point_2d_homogeneous[:2] / point_2d_homogeneous[2]

            if 0 <= point_2d[0] and point_2d[0] < cv_image.shape[1] and 0 <= point_2d[1] and point_2d[1] < cv_image.shape[0] and point_camera_frame[2] > 0:
                point_2d = (int(point_2d[0]), int(point_2d[1]))
                # Draw the point on the image
                color = (0, 255, 255)  # BGR format

                # Coordinates of the center and radius
                center = point_2d
                radius = 10 

                # Draw the circle part of the teardrop
                cv2.circle(cv_image, center, radius, color, -1)

                # Draw the triangle part of the teardrop
                points = np.array([
                    [center[0], center[1] + 2*radius],
                    [center[0] - radius, center[1]],
                    [center[0] + radius, center[1]]
                ], np.int32)
                points = points.reshape((-1, 1, 2))
                cv2.fillPoly(cv_image, [points], color)

            else:
                self.get_logger().info('Projected point is outside the image boundaries or clicked outside PointCloud Data\n')

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
