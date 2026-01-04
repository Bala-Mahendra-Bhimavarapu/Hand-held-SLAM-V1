#!/usr/bin/env python3
"""Sensors Only Launch - Test all sensors without SLAM"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Static TF Broadcaster
    static_tf_node = Node(
        package='slam_fusion',
        executable='static_tf_broadcaster',
        name='static_tf_broadcaster',
        output='screen'
    )

    # MPU6050 IMU
    mpu6050_node = Node(
        package='slam_sensors',
        executable='mpu6050_node',
        name='mpu6050_imu',
        output='screen',
        parameters=[{
            'i2c_bus': 1,
            'i2c_address': 0x68,
            'frame_id': 'imu_link',
            'rate': 100
        }]
    )

    # IMU Filter
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'fixed_frame': 'base_link'
        }],
        remappings=[
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/data', '/imu/data')
        ]
    )

    # Pi AI Camera
    pi_ai_cam_node = Node(
        package='slam_sensors',
        executable='pi_ai_cam_node',
        name='pi_ai_cam',
        output='screen',
        parameters=[{
            'frame_id': 'camera_link',
            'width': 1280,
            'height': 720,
            'fps': 30
        }]
    )

    # Arducam ToF Camera WITH YOUR CUSTOM CALIBRATION
    arducam_tof_node = Node(
        package='slam_sensors',
        executable='arducam_tof_node',
        name='arducam_tof',
        output='screen',
        parameters=[{
            'frame_id': 'tof_link',
            'fps': 20,
            'max_distance': 4000,
            'min_distance': 200,
            'confidence_threshold': 30,
            # Custom calibration
            'use_custom_calibration': True,
            'fx': 215.15,
            'fy': 214.79,
            'cx': 115.59,
            'cy': 86.71,
            'k1': 3.8097039893848543,
            'k2': -110.7877471544198,
            'p1': 0.0028822716027511845,
            'p2': -0.011241297709775491,
            'k3': 348.33576851921447
        }]
    )

    # YOLO Detector
    yolo_detector_node = Node(
        package='slam_landmarks',
        executable='yolo_detector_node',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model_path': '/home/stargate/slam_ws/models/yolov8n.pt',
            'confidence_threshold': 0.6,
            'iou_threshold': 0.5,
            'landmark_classes': ['bottle', 'chair', 'potted plant', 'tv', 'laptop']
        }]
    )

    # 3D Landmark Association
    landmark_3d_node = Node(
        package='slam_landmarks',
        executable='landmark_3d_node',
        name='landmark_3d',
        output='screen',
        parameters=[{
            'max_depth': 4.0,
            'min_depth': 0.3
        }]
    )

    return LaunchDescription([
        static_tf_node,
        mpu6050_node,
        imu_filter_node,
        pi_ai_cam_node,
        arducam_tof_node,
        yolo_detector_node,
        landmark_3d_node
    ])
