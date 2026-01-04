#!/usr/bin/env python3
"""YOLO Detector Node"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

class YOLODetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Parameters
        self.declare_parameter('model_path', '/home/stargate/slam_ws/models/yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('iou_threshold', 0.5)
        self.declare_parameter('landmark_classes', ['bottle', 'chair', 'potted plant', 'tv', 'laptop'])

        self.bridge = CvBridge()

        # Load model
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.landmark_classes = self.get_parameter('landmark_classes').value

        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'Loaded YOLO: {model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO: {e}')
            raise

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.detections_pub = self.create_publisher(Detection2DArray, '/camera/detections', 10)
        self.annotated_pub = self.create_publisher(Image, '/camera/detections/annotated', 10)

        self.get_logger().info('YOLO detector started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run detection
            results = self.model(cv_image, conf=self.confidence_threshold, iou=self.iou_threshold, verbose=False)
     
            # Create detection array
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header
            
            detected_classes = [] # Track what we see
            
            for result in results:
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    class_name = result.names[class_id]
                    
                    detected_classes.append(class_name) # Add to list

                    if class_name not in self.landmark_classes:
                        self.get_logger().info(f'SKIPPING: {class_name} (not in landmark_classes)')
                        continue

                    detection = Detection2D()
                    detection.header = msg.header

                    # Hypothesis
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = class_name
                    hypothesis.hypothesis.score = float(box.conf[0])
                    detection.results.append(hypothesis)

                    # Bounding box
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    detection.bbox.center.position.x = float((x1 + x2) / 2)
                    detection.bbox.center.position.y = float((y1 + y2) / 2)
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)

                    detections_msg.detections.append(detection)
            if len(detected_classes) > 0:
                self.get_logger().info(f'Detected: {detected_classes}, Published: {len(detections_msg.detections)} landmarks')

            self.detections_pub.publish(detections_msg)

            # Publish annotated image
            annotated = results[0].plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
