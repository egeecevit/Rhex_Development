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
        self.camera_offset = np.array([-0.0075, 0.0105, 0.025])  

        self.robot_pose = None
        self.static_point_world_frame = None
        self.fov_horizontal = 60  # in degrees
        self.fov_vertical = 45  # in degrees
        self.start_time = time.time()
        self.amplitude = 0.02
        self.frequency = 3

        self.odom_timestamp = None
        self.br = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info(f'Mark point node has started with camera matrix: \n{self.camera_matrix}')



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

    def listen_transform(self):
        if self.tf_buffer.can_transform('camera_depth_frame', 'world', rclpy.time.Time(seconds=0)):
            try:
                transform = self.tf_buffer.lookup_transform('camera_depth_frame', 'world', rclpy.time.Time(seconds=0))
                point_stamped = PointStamped()
                point_stamped.header.frame_id = 'world'
                point_stamped.point.x = self.static_point_world_frame[0]
                point_stamped.point.y = self.static_point_world_frame[1]
                point_stamped.point.z = self.static_point_world_frame[2]
                point_in_camera_frame_stamped = do_transform_point(point_stamped, transform)
                point_in_camera_frame = np.array([point_in_camera_frame_stamped.point.x,
                                                  point_in_camera_frame_stamped.point.y,
                                                  point_in_camera_frame_stamped.point.z])
                self.get_logger().info(f'Point in camera frame: {point_in_camera_frame}\n')

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

        if self.static_point_world_frame is None:
            point_3d_robot = np.array([1.0, 0.0, 0.0])  # Only x, y, z
            self.static_point_world_frame = point_3d_robot + np.array([self.robot_pose.position.x, 
                                                                        self.robot_pose.position.y, 
                                                                        self.robot_pose.position.z]) + self.camera_offset
            
        #self.get_logger().info(f'Static point in world frame: {self.static_point_world_frame}\n')
        #self.get_logger().info(f'Received robot pose: {self.robot_pose.position.x}, {self.robot_pose.position.y}, {self.robot_pose.position.z}\n')

    def move_marker(self, point_in_camera_frame):
        elapsed_time = time.time() - self.start_time
        y = self.amplitude * np.sin(self.frequency * elapsed_time) + point_in_camera_frame[1]
        point_in_camera_frame[1] = y



    def image_callback(self, msg):
        if self.robot_pose is None or self.static_point_world_frame is None:
            self.get_logger().info('Robot pose or static point not yet received\n')
            return

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        point_in_camera_frame = self.listen_transform()

        if point_in_camera_frame is not None:
            # Project the 3D point to 2D using the camera intrinsic matrix
            self.move_marker(point_in_camera_frame)
            point_2d_homogeneous = self.camera_matrix @ point_in_camera_frame
            self.get_logger().info(f'Point 2D homogeneous before normalization: {point_2d_homogeneous}\n')

            # Normalize the 2D coordinates by dividing by the third coordinate
            point_2d = point_2d_homogeneous[:2] / point_2d_homogeneous[2]
            self.get_logger().info(f'Projected 2D point: {point_2d}\n')
            

            # # Check if the point is within the image boundaries
            if 0 <= point_2d[0] and point_2d[0] < cv_image.shape[1] and 0 <= point_2d[1] and point_2d[1] < cv_image.shape[0] and point_in_camera_frame[2] > 0:
                point_2d = (int(point_2d[0]), int(point_2d[1]))
                # Draw the point on the image
                color = (0, 255, 255)  # BGR format

                # Coordinates of the center and radius
                center = point_2d
                radius = 10  # Adjust the radius as needed

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
