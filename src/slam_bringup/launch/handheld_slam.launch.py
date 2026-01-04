#!/usr/bin/env python3
"""
Complete Handheld SLAM Launch File
Pi AI Camera = PRIMARY SLAM sensor
Arducam ToF = SUPPORT depth sensor
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Static TF
    static_tf_node = Node(
        package='slam_fusion',
        executable='static_tf_broadcaster',
        name='static_tf_broadcaster',
        output='screen'
    )
    
    # IMU
    imu_node = Node(
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
            'world_frame': 'enu'
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/data', '/imu/data')
        ]
    )
    
    # PRIMARY: Pi AI Camera
    pi_cam_node = Node(
        package='slam_sensors',
        executable='pi_ai_cam_node',
        name='pi_ai_cam',
        output='screen',
        parameters=[{
            'camera_id': 0,
            'width': 1280,
            'height': 720,
            'fps': 30,
            'frame_id': 'camera_link',
            # Pi AI Camera Calibration (1280 x 720)
            'fx': 974.82,
            'fy': 977.03,
            'cx': 630.57,
            'cy': 371.11,
            'k1': -6.252192720771978,
            'k2': 13.495239229048028,
            'p1': 0.0034537142110085354,
            'p2': -0.003523666424911846,
            'k3': 8.770640895609361
        }]
    )
    
    # SUPPORT: Arducam ToF
    tof_node = Node(
        package='slam_sensors',
        executable='arducam_tof_node',
        name='arducam_tof',
        output='screen',
        parameters=[{
            'frame_id': 'tof_link',
            'fps': 20,
            'max_depth': 4000,
            'min_depth': 200,
            'confidence_threshold': 30
            'config_file': '', # Optional: '/path/to/config.cfg'

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
    
    # RTAB-Map Visual SLAM (Primary on RGB camera)
    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'queue_size': 30,
            
            # Memory
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            
            # Visual SLAM priority
            'RGBD/NeighborLinkRefining': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/AngularUpdate': '0.01',
            'RGBD/LinearUpdate': '0.01',
            
            # 3D mode
            'Optimizer/Slam2D': 'false',
            'Reg/Strategy': '0',  # 0=Visual primary
            
            # Visual features (primary)
            'Vis/MinInliers': '20',
            'Vis/InlierDistance': '0.1',
            'Vis/MaxDepth': '4.0',
            'Vis/FeatureType': '6',
            'Vis/MaxFeatures': '1000',
            
            # ICP (support from ToF)
            'Icp/VoxelSize': '0.05',
            'Icp/MaxCorrespondenceDistance': '0.2',
            
            # Grid
            'Grid/FromDepth': 'true',
            'Grid/CellSize': '0.05',
            'Grid/RangeMax': '4.0'
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/tof/depth/image_raw'),
            ('depth/camera_info', '/tof/camera_info'),
            ('scan_cloud', '/tof/points'),
            ('imu', '/imu/data')
        ],
        arguments=['--delete_db_on_start']
    )
    
    # YOLO Landmark Detector
    yolo_node = Node(
        package='slam_landmarks',
        executable='yolo_detector_node',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model_path': '~/slam_ws/models/yolov8n.pt',
            'confidence_threshold': 0.6,
            'landmark_classes': ['bottle', 'chair', 'potted plant', 'tv', 'laptop', 'book']
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
        imu_node,
        imu_filter_node,
        pi_cam_node,
        tof_node,
        rtabmap_node,
        yolo_node,
        landmark_3d_node
    ])
