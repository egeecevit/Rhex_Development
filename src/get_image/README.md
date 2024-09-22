# AR Marking With PC2 Data
## Task Description
This task is about matching and visualizing a clicked point from Cv2 window to PointCloud2 data which can be used for landmarking or checkpointing purposes.
### Steps To Implement
**1.** Camera feed is gathered from the simulation environment from `/depth_camera/image_raw` topic and visualized using CvBridge and Cv2.

**2.** PointCloud2 data is gathered from `/depth_camera/points` topic.

**3.** A mouse click event is registered on the Cv2 window and the clicked point is stored.

**4.** The clicked point is then matched with the PointCloud2 data.

**5.** A world frame is broadcasted which is the parent of all other frames. Which will allow us to transform the point from the world frame to the robot frame or vice a versa.

**6.** The matched point is then transformed into world coordinate frame and published to `/rhex_mark` topic. To check whether the point is correctly transformed using Rviz2.

**7.** To be able to get the dynamic movement of the point, point is transformed from world coordinate frame to camera frame.

**8.** The point is then projected to the image plane using the camera intrinsic matrix.


