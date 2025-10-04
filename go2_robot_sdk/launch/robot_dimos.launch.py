#!/usr/bin/env python3
"""
DimOS-compatible launch file with optimized Go2 SDK components.
Integrates optimized lidar processing with dimos-unitree framework.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for DimOS-compatible optimized Go2 SDK."""
    
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.123.161',
        description='IP address of the Go2 robot'
    )
    
    conn_type_arg = DeclareLaunchArgument(
        'conn_type',
        default_value='webrtc',
        description='Connection type: webrtc or wifi'
    )
    
    use_optimized_lidar_arg = DeclareLaunchArgument(
        'use_optimized_lidar',
        default_value='true',
        description='Use optimized C++ lidar processor instead of Python version'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Configuration
    robot_ip = LaunchConfiguration('robot_ip')
    conn_type = LaunchConfiguration('conn_type')
    use_optimized_lidar = LaunchConfiguration('use_optimized_lidar')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Robot SDK node with dimos-compatible topics
    go2_driver_node = Node(
        package='go2_robot_sdk',
        executable='go2_driver_node',
        name='go2_driver_node',
        output='screen',
        parameters=[
            {'robot_ip': robot_ip},
            {'conn_type': conn_type},
            {'use_sim_time': use_sim_time},
            # DimOS-compatible topic mappings
            {'odom_topic': '/odom'},
            {'cmd_vel_topic': '/cmd_vel_out'},
            {'pose_cmd_topic': '/pose_cmd'},
            {'go2_states_topic': '/go2_states'},
            {'imu_topic': '/imu'},
            {'webrtc_topic': '/webrtc_req'},
            {'camera_topic': '/camera/image_raw'},
            {'camera_compressed_topic': '/camera/compressed'},
            {'camera_info_topic': '/camera/camera_info'},
        ]
    )
    
    # Optimized C++ lidar processor (DimOS-compatible)
    optimized_lidar_node = Node(
        package='lidar_processor_cpp',
        executable='lidar_processor_node',
        name='lidar_processor_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            # Performance optimizations enabled
            {'enable_memory_pool': True},
            {'pool_size': 1000},
            {'enable_spatial_hashing': True},
            {'hash_resolution': 0.1},
            {'cleanup_strategy': 'fifo'},
            {'max_points_per_cell': 50},
            # DimOS-compatible topics
            {'input_cloud_topic': '/lidar/points'},
            {'output_cloud_topic': '/lidar/filtered'},
            {'costmap_topic': '/local_costmap/costmap'},
        ],
        condition=IfCondition(use_optimized_lidar)
    )
    
    # Fallback Python lidar processor for comparison
    python_lidar_node = Node(
        package='lidar_processor',
        executable='lidar_processor_node',
        name='lidar_processor_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'input_cloud_topic': '/lidar/points'},
            {'output_cloud_topic': '/lidar/filtered'},
            {'costmap_topic': '/local_costmap/costmap'},
        ],
        condition=IfCondition(PythonExpression(['not ', use_optimized_lidar]))
    )
    
    # Speech processor for dimos integration
    speech_processor_node = Node(
        package='speech_processor',
        executable='tts_node',
        name='tts_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'webrtc_topic': '/webrtc_req'},
        ]
    )
    
    # COCO detector for object detection (dimos integration)
    coco_detector_node = Node(
        package='coco_detector',
        executable='coco_detector_node',
        name='coco_detector_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'camera_topic': '/camera/image_raw'},
            {'detection_topic': '/detections'},
        ]
    )
    
    # Static transform publisher for sensor frames (dimos-compatible)
    base_to_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0', '0', '0.44', '0', '0', '0', 'base_link', 'lidar_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0.1', '0', '0.35', '0', '0.1', '0', 'base_link', 'camera_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Map to odom transform for navigation
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Launch description
    return LaunchDescription([
        # Arguments
        robot_ip_arg,
        conn_type_arg,
        use_optimized_lidar_arg,
        use_sim_time_arg,
        
        # Log startup message
        LogInfo(msg='Starting DimOS-compatible optimized Go2 SDK'),
        LogInfo(msg=[
            'Robot IP: ', robot_ip, 
            ', Connection: ', conn_type,
            ', Optimized Lidar: ', use_optimized_lidar
        ]),
        
        # Core nodes
        go2_driver_node,
        optimized_lidar_node,
        python_lidar_node,
        speech_processor_node,
        coco_detector_node,
        
        # Transform publishers
        base_to_lidar_tf,
        base_to_camera_tf,
        map_to_odom_tf,
    ])