#!/usr/bin/env python3
"""3D Landmark Association - combines YOLO detections with ToF depth using spatial proximity"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import PointCloud2, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import tf2_ros
import tf2_geometry_msgs

class Landmark3DNode(Node):
    def __init__(self):
        super().__init__('landmark_3d_node')

        self.declare_parameter('max_depth', 4.0)
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('detection_cone_angle', 0.3)  # radians (~17 degrees)

        self.max_depth = self.get_parameter('max_depth').value
        self.min_depth = self.get_parameter('min_depth').value
        self.cone_angle = self.get_parameter('detection_cone_angle').value

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Cached data
        self.latest_pointcloud = None
        self.latest_camera_info = None

        # Publishers
        self.landmark_pub = self.create_publisher(MarkerArray, '/landmarks_3d', 10)

        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/camera/detections', self.detection_callback, 10)
        self.pc_sub = self.create_subscription(
            PointCloud2, '/tof/points', self.pointcloud_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_callback, 10)

        self.get_logger().info('3D Landmark association node started (point cloud mode)')

    def pointcloud_callback(self, msg):
        self.latest_pointcloud = msg

    def info_callback(self, msg):
        self.latest_camera_info = msg

    def detection_callback(self, msg):
        self.get_logger().info(f'Processing {len(msg.detections)} detections')
        if self.latest_pointcloud is None or self.latest_camera_info is None:
            return

        try:
            # Convert point cloud to numpy array
            points = []
            for point in pc2.read_points(self.latest_pointcloud, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2]])

            if len(points) == 0:
                self.get_logger().warn('Empty point cloud')
                return

            points = np.array(points)

            # Create MarkerArray to hold all landmarks
            marker_array = MarkerArray()
            marker_id = 0

            for detection in msg.detections:
                # Get detection center direction
                u = detection.bbox.center.position.x
                v = detection.bbox.center.position.y

                # Camera intrinsics
                fx = self.latest_camera_info.k[0]
                fy = self.latest_camera_info.k[4]
                cx = self.latest_camera_info.k[2]
                cy = self.latest_camera_info.k[5]

                # Ray direction in camera frame (normalized)
                ray_x = (u - cx) / fx
                ray_y = (v - cy) / fy
                ray_z = 1.0
                ray_norm = np.sqrt(ray_x**2 + ray_y**2 + ray_z**2)
                ray_dir = np.array([ray_x / ray_norm, ray_y / ray_norm, ray_z / ray_norm])

                # Find points within cone around this direction
                point_dirs = points / np.linalg.norm(points, axis=1, keepdims=True)
                angles = np.arccos(np.clip(np.dot(point_dirs, ray_dir), -1.0, 1.0))

                # Filter by cone angle and depth range
                distances = np.linalg.norm(points, axis=1)
                valid_mask = (angles < self.cone_angle) & (distances > self.min_depth) & (distances < self.max_depth)
                valid_points = points[valid_mask]

                if len(valid_points) == 0:
                    self.get_logger().warn(f'No points in detection cone (checked {len(points)} points)')
                    continue

                # Use median point (robust to outliers)
                landmark_point = np.median(valid_points, axis=0)

                # Create point in camera frame
                point_stamped = PointStamped()
                point_stamped.header.frame_id = self.latest_camera_info.header.frame_id
                point_stamped.header.stamp = self.get_clock().now().to_msg()
                point_stamped.point.x = float(landmark_point[0])
                point_stamped.point.y = float(landmark_point[1])
                point_stamped.point.z = float(landmark_point[2])

                # Transform to base_link
                try:
                    transform = self.tf_buffer.lookup_transform(
                        'base_link',
                        point_stamped.header.frame_id,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1))

                    map_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)

                    class_name = detection.results[0].hypothesis.class_id if detection.results else "unknown"

                    # Create arrow marker
                    marker = Marker()
                    marker.header.frame_id = 'base_link'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = 'landmarks'
                    marker.id = marker_id
                    marker.type = Marker.ARROW
                    marker.action = Marker.ADD
                    marker.pose.position = map_point.point
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.3  # Arrow length
                    marker.scale.y = 0.3  # Arrow width
                    marker.scale.z = 0.3  # Arrow height
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0
                    marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
                    marker_array.markers.append(marker)

                    # Create text label
                    text_marker = Marker()
                    text_marker.header.frame_id = 'base_link'
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.ns = 'labels'
                    text_marker.id = marker_id + 1000
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    text_marker.pose.position = map_point.point
                    text_marker.pose.position.z += 0.2  # Place text above arrow
                    text_marker.text = class_name
                    text_marker.scale.z = 1.0  # Text size
                    text_marker.color.r = 1.0
                    text_marker.color.g = 1.0
                    text_marker.color.b = 1.0
                    text_marker.color.a = 1.0
                    text_marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
                    marker_array.markers.append(text_marker)

                    marker_id += 1

                    self.get_logger().info(
                        f'Added {class_name}: [{landmark_point[0]:.2f}, {landmark_point[1]:.2f}, {landmark_point[2]:.2f}] '
                        f'({len(valid_points)} points in cone)'
                    )

                except Exception as e:
                    self.get_logger().warn(f'TF transform failed: {e}')

            # Publish all landmarks at once
            if len(marker_array.markers) > 0:
                self.landmark_pub.publish(marker_array)
                self.get_logger().info(f'Published {len(marker_array.markers)//2} landmarks with labels')

        except Exception as e:
            self.get_logger().error(f'Association error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Landmark3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
