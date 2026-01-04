#!/usr/bin/env python3
"""
Pi AI Camera ROS2 Node - PRIMARY SLAM SENSOR
Uses rpicam-vid for high-framerate RGB streaming
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import subprocess
import cv2
import numpy as np
from cv_bridge import CvBridge
import threading
import time

class PiAICameraNode(Node):
    def __init__(self):
        super().__init__('pi_ai_camera_node')
        
        # Parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 1280)  
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)  # Higher FPS for visual odometry
        self.declare_parameter('frame_id', 'camera_link')
        
        # Camera calibration (USE YOUR INTRINSICS HERE)
        self.declare_parameter('fx', 974.82)
        self.declare_parameter('fy', 977.03)
        self.declare_parameter('cx', 630.57)
        self.declare_parameter('cy', 371.11)
        self.declare_parameter('k1', -6.252192720771978)
        self.declare_parameter('k2', 13.495239229048028)
        self.declare_parameter('p1', 0.0034537142110085354)
        self.declare_parameter('p2', -0.003523666424911846)
        self.declare_parameter('k3', 8.770640895609361)
        
        self.camera_id = self.get_parameter('camera_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Get calibration
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        self.dist = [
            self.get_parameter('k1').value,
            self.get_parameter('k2').value,
            self.get_parameter('p1').value,
            self.get_parameter('p2').value,
            self.get_parameter('k3').value
        ]
        
        self.bridge = CvBridge()
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # Start rpicam stream
        self.stream_port = 8888
        self.start_rpicam_stream()
        time.sleep(2)
        
        # Capture thread
        self.running = True
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()
        
        self.get_logger().info(f'Pi AI Camera (PRIMARY SLAM) started at {self.fps} Hz')
        self.get_logger().info(f'Resolution: {self.width}x{self.height}')
    
    def start_rpicam_stream(self):
        """Start rpicam-vid UDP stream"""
        cmd = [
            'rpicam-vid',
            '--camera', str(self.camera_id),
            '--width', str(self.width),
            '--height', str(self.height),
            '--framerate', str(self.fps),
            '--inline',
            '-o', f'udp://127.0.0.1:{self.stream_port}',  # Changed to UDP
            '-t', '0',
            '-n',
            '--codec', 'mjpeg'  # Changed to MJPEG for better compatibility
        ]
        
        self.rpicam_process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        
        self.get_logger().info(f'rpicam-vid started (PID: {self.rpicam_process.pid})')
    
    def capture_loop(self):
        """Capture loop"""
        stream_url = f'udp://127.0.0.1:{self.stream_port}' # UDP instead of TCP
        
        # Wait for stream to be ready
        max_retries = 5
        retry_count = 0
        cap = None
        
        while retry_count < max_retries and self.running:
            cap = cv2.VideoCapture(stream_url, cv2.CAP_FFMPEG)
            if cap.isOpened():
                self.get_logger().info('Stream opened successfully')
                break
            
            retry_count += 1
            self.get_logger().warn(f'Stream not ready, retry {retry_count}/{max_retries}...')
            time.sleep(1)
            
            if cap is not None:
                cap.release()
        
        if not cap or not cap.isOpened():
            self.get_logger().error('Failed to open stream after retries')
            self.get_logger().error(f'Tried URL: {stream_url}')
            self.get_logger().error('Check: 1) Camera connected? 2) rpicam-vid running?')
            return
        
        frame_count = 0
        consecutive_failures = 0
        
        while self.running and rclpy.ok():
            ret, frame = cap.read()
            
            if not ret:
                consecutive_failures += 1
                if consecutive_failures > 30:  # 1 second at 30fps
                    self.get_logger().error('Too many consecutive frame failures')
                    break
                continue
            
            consecutive_failures = 0
            
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height))
            
            stamp = self.get_clock().now().to_msg()
            
            # Publish image
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = self.frame_id
            self.image_pub.publish(img_msg)
            
            # Publish camera info
            info_msg = self.create_camera_info(stamp)
            self.info_pub.publish(info_msg)
            
            frame_count += 1
            
            if frame_count % 300 == 0:
                self.get_logger().info(f'Published {frame_count} frames')
        
        cap.release()
        self.get_logger().info('Capture loop ended')
    
    def create_camera_info(self, stamp):
        """Create calibrated camera info"""
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = self.width
        msg.height = self.height
        
        msg.k = [self.fx, 0.0, self.cx,
                 0.0, self.fy, self.cy,
                 0.0, 0.0, 1.0]
        
        msg.d = self.dist
        msg.distortion_model = 'plumb_bob'
        
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        
        msg.p = [self.fx, 0.0, self.cx, 0.0,
                 0.0, self.fy, self.cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        
        return msg
    
    def destroy_node(self):
        self.running = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join(timeout=2)
        if hasattr(self, 'rpicam_process'):
            self.rpicam_process.terminate()
            self.rpicam_process.wait(timeout=2)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PiAICameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
