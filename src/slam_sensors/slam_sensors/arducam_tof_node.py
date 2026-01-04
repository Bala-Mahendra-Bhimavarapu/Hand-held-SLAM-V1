#!/usr/bin/env python3
"""
Arducam ToF ROS2 Node - SUPPORT DEPTH SENSOR
With custom calibration support
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header
import cv2
import numpy as np
from cv_bridge import CvBridge
import struct
import sys

try:
    import ArducamDepthCamera as ac
except ImportError:
    print("ERROR: ArducamDepthCamera SDK not installed!")
    sys.exit(1)

class ArducamToFNode(Node):
    def __init__(self):
        super().__init__('arducam_tof_node')

        # Parameters
        self.declare_parameter('frame_id', 'tof_link')
        self.declare_parameter('fps', 20)
        self.declare_parameter('max_distance', 4000)
        self.declare_parameter('min_distance', 200)
        self.declare_parameter('confidence_threshold', 30)

        # Custom calibration parameters
        self.declare_parameter('use_custom_calibration', True)
        self.declare_parameter('fx', 215.15)
        self.declare_parameter('fy', 214.79)
        self.declare_parameter('cx', 115.59)
        self.declare_parameter('cy', 86.71)
        self.declare_parameter('k1', 3.8097039893848543)
        self.declare_parameter('k2', -110.7877471544198)
        self.declare_parameter('p1', 0.0028822716027511845)
        self.declare_parameter('p2', -0.011241297709775491)
        self.declare_parameter('k3', 348.33576851921447)

        self.frame_id = self.get_parameter('frame_id').value
        self.fps = self.get_parameter('fps').value
        self.max_distance = self.get_parameter('max_distance').value
        self.min_distance = self.get_parameter('min_distance').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.use_custom_calibration = self.get_parameter('use_custom_calibration').value

        self.bridge = CvBridge()

        # Publishers
        self.depth_pub = self.create_publisher(Image, '/tof/depth/image_raw', 10)
        self.confidence_pub = self.create_publisher(Image, '/tof/confidence/image_raw', 10)
        self.depth_colored_pub = self.create_publisher(Image, '/tof/depth/image_colored', 10)
        self.points_pub = self.create_publisher(PointCloud2, '/tof/points', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/tof/camera_info', 10)

        # Initialize camera
        try:
            self.cam = ac.ArducamCamera()
            ret = self.cam.open(ac.Connection.CSI, 0)
            if ret != 0:
                raise RuntimeError(f'Camera open failed: {ret}')

            ret = self.cam.start(ac.FrameType.DEPTH)
            if ret != 0:
                self.cam.close()
                raise RuntimeError(f'Camera start failed: {ret}')

            self.cam.setControl(ac.Control.RANGE, self.max_distance)
            actual_range = self.cam.getControl(ac.Control.RANGE)
            self.get_logger().info(f'Camera range set to: {actual_range}mm')
            
            self.info = self.cam.getCameraInfo()
            self.width = self.info.width
            self.height = self.info.height
            self.device_type = self.info.device_type

            # Setup calibration
            if self.use_custom_calibration:
                self.fx = self.get_parameter('fx').value
                self.fy = self.get_parameter('fy').value
                self.cx = self.get_parameter('cx').value
                self.cy = self.get_parameter('cy').value
                self.k1 = self.get_parameter('k1').value
                self.k2 = self.get_parameter('k2').value
                self.p1 = self.get_parameter('p1').value
                self.p2 = self.get_parameter('p2').value
                self.k3 = self.get_parameter('k3').value

                self.get_logger().info('Using CUSTOM calibration')
                self.get_logger().info(f'  fx={self.fx:.2f}, fy={self.fy:.2f}')
                self.get_logger().info(f'  cx={self.cx:.2f}, cy={self.cy:.2f}')
            else:
                # Use SDK calibration
                self.fx = self.info.fx
                self.fy = self.info.fy
                self.cx = self.info.cx
                self.cy = self.info.cy
                self.k1 = 0.0
                self.k2 = 0.0
                self.p1 = 0.0
                self.p2 = 0.0
                self.k3 = 0.0

                self.get_logger().info('Using SDK calibration')
                self.get_logger().info(f'  fx={self.fx:.2f}, fy={self.fy:.2f}')
                self.get_logger().info(f'  cx={self.cx:.2f}, cy={self.cy:.2f}')

            self.get_logger().info(f'ToF Camera (SUPPORT) initialized')
            self.get_logger().info(f'  SDK version: {ac.__version__}')
            self.get_logger().info(f'  Resolution: {self.width}x{self.height}')
            self.get_logger().info(f'  Device type: {self.device_type}')
            self.get_logger().info(f'  Frame rate: {self.fps} Hz')

        except Exception as e:
            self.get_logger().error(f'Initialization failed: {e}')
            raise

        self.timer = self.create_timer(1.0 / self.fps, self.capture_callback)
        self.frame_count = 0

    def capture_callback(self):
        try:
            frame = self.cam.requestFrame(2000)

            if frame is None:
                self.get_logger().warn('Frame is None')
                return

            if not isinstance(frame, ac.DepthData):
                self.get_logger().warn(f'Unexpected frame type: {type(frame)}')
                self.cam.releaseFrame(frame)
                return

            depth_data = frame.depth_data
            confidence_data = frame.confidence_data

            if depth_data is None:
                self.get_logger().warn('Depth data is None')
                self.cam.releaseFrame(frame)
                return

            depth = depth_data.copy()
            confidence = confidence_data.copy() if confidence_data is not None else None

            stamp = self.get_clock().now().to_msg()

            # Filter based on confidence and distance
            if confidence is not None:
                depth = np.nan_to_num(depth)
                mask = (confidence >= self.confidence_threshold) & \
                       (depth >= self.min_distance) & \
                       (depth <= self.max_distance)
                depth_filtered = depth.copy()
                depth_filtered[~mask] = 0
            else:
                depth = np.nan_to_num(depth)
                mask = (depth >= self.min_distance) & (depth <= self.max_distance)
                depth_filtered = depth.copy()
                depth_filtered[~mask] = 0

            # Publish raw depth image (16-bit, millimeters)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_filtered.astype(np.uint16), encoding='16UC1')
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = self.frame_id
            self.depth_pub.publish(depth_msg)

            # Publish colored depth visualization
            depth_colored = (depth_filtered * (255.0 / self.max_distance)).astype(np.uint8)
            depth_colored = cv2.applyColorMap(depth_colored, cv2.COLORMAP_RAINBOW)
            depth_colored[depth_filtered == 0] = (0, 0, 0)  # Black for invalid

            colored_msg = self.bridge.cv2_to_imgmsg(depth_colored, encoding='bgr8')
            colored_msg.header.stamp = stamp
            colored_msg.header.frame_id = self.frame_id
            self.depth_colored_pub.publish(colored_msg)

            # Publish confidence image if available
            if confidence is not None and self.device_type == ac.DeviceType.VGA:
                confidence_normalized = confidence.copy()
                cv2.normalize(confidence_normalized, confidence_normalized, 0, 255, cv2.NORM_MINMAX)
                confidence_msg = self.bridge.cv2_to_imgmsg(confidence_normalized.astype(np.uint8), encoding='mono8')
                confidence_msg.header.stamp = stamp
                confidence_msg.header.frame_id = self.frame_id
                self.confidence_pub.publish(confidence_msg)

            # Publish point cloud
            points_msg = self.depth_to_pointcloud(depth_filtered, stamp)
            self.points_pub.publish(points_msg)

            # Publish camera info
            info_msg = self.create_camera_info(stamp)
            self.info_pub.publish(info_msg)

            # Release frame
            self.cam.releaseFrame(frame)

            self.frame_count += 1
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')

        except Exception as e:
            self.get_logger().error(f'Capture error: {e}')

    def depth_to_pointcloud(self, depth, stamp):
        """Convert depth image to point cloud"""
        points = []

        # Downsample for performance (every 2nd pixel)
        for v in range(0, self.height, 2):
            for u in range(0, self.width, 2):
                d = depth[v, u]
                if d == 0 or d < self.min_distance or d > self.max_distance:
                    continue

                # Convert mm to meters
                z = d / 1000.0

                # Back-project to 3D using pinhole camera model
                x = (u - self.cx) * z / self.fx
                y = (v - self.cy) * z / self.fy

                points.append([x, y, z])

        # Create PointCloud2 message
        header = Header()
        header.stamp = stamp
        header.frame_id = self.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Pack points into binary data
        points_struct = b''.join([struct.pack('fff', *p) for p in points])

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = False
        msg.is_bigendian = False
        msg.fields = fields
        msg.point_step = 12
        msg.row_step = msg.point_step * len(points)
        msg.data = points_struct

        return msg

    def create_camera_info(self, stamp):
        """Create CameraInfo message with calibration"""
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = self.width
        msg.height = self.height

        # Camera intrinsic matrix K
        msg.k = [self.fx, 0.0, self.cx,
                 0.0, self.fy, self.cy,
                 0.0, 0.0, 1.0]

        # Distortion coefficients [k1, k2, p1, p2, k3]
        msg.d = [self.k1, self.k2, self.p1, self.p2, self.k3]
        msg.distortion_model = 'plumb_bob'

        # Rectification matrix (identity for monocular)
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]

        # Projection matrix P
        msg.p = [self.fx, 0.0, self.cx, 0.0,
                 0.0, self.fy, self.cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]

        return msg

    def destroy_node(self):
        """Cleanup camera resources"""
        self.get_logger().info('Shutting down camera...')
        try:
            if hasattr(self, 'cam'):
                self.cam.stop()
                self.cam.close()
                self.get_logger().info('Camera closed successfully')
        except Exception as e:
            self.get_logger().error(f'Error closing camera: {e}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArducamToFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
