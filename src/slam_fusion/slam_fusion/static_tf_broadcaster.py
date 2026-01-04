#!/usr/bin/env python3
"""Static TF Broadcaster"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()

    def publish_static_transforms(self):
        static_transforms = []

        # base_link -> imu_link
        t1 = TransformStamped()
        t1.header.frame_id = 'base_link'
        t1.child_frame_id = 'imu_link'
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.055
        t1.transform.rotation.w = 1.0
        static_transforms.append(t1)

        # base_link -> camera_link
        t2 = TransformStamped()
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'camera_link'
        t2.transform.translation.x = 0.017
        t2.transform.translation.y = -0.027
        t2.transform.translation.z = 0.025
        t2.transform.rotation.w = 1.0
        static_transforms.append(t2)

        # base_link -> tof_link
        t3 = TransformStamped()
        t3.header.frame_id = 'base_link'
        t3.child_frame_id = 'tof_link'
        t3.transform.translation.x = 0.013
        t3.transform.translation.y = 0.016
        t3.transform.translation.z = 0.027
        t3.transform.rotation.w = 1.0
        static_transforms.append(t3)

        for t in static_transforms:
            t.header.stamp = self.get_clock().now().to_msg()

        self.broadcaster.sendTransform(static_transforms)
        self.get_logger().info('Static TF published')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
