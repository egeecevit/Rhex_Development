import rclpy
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np

class PointCloudVisualizer(Node):
    def __init__(self):
        super().__init__('pointcloud_visualizer')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth_camera/points',
            self.pointcloud_callback,
            10
        )
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='Point Cloud Visualizer', width=800, height=600)
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)
        self.vis.get_render_option().background_color = np.asarray([0, 0, 0])  # Set background to black
        self.initialized = False

    def pointcloud_callback(self, msg):
        points = []
        for point in point_cloud2.read_points(msg, skip_nans=True):
            points.append([point[0], point[1], point[2]])

        if points:
            points_np = np.array(points)
            print(f"Points shape: {points_np.shape}")  # Debugging print
            self.pcd.points = o3d.utility.Vector3dVector(points_np)

            if not self.initialized:
                self.vis.add_geometry(self.pcd)
                self.initialized = True

            self.vis.update_geometry(self.pcd)
            self.vis.poll_events()
            self.vis.update_renderer()

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudVisualizer()
    rclpy.spin(node)
    node.vis.destroy_window()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
