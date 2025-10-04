#!/usr/bin/env python3
"""
DimOS Integration Bridge

This module provides integration between optimized Go2 SDK components and the dimos-unitree framework.
It bridges our high-performance C++ lidar processing with DimOS spatial memory, navigation, and perception systems.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from typing import Optional, Dict, Any, Callable
import threading
import time

# ROS message types
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from go2_interfaces.msg import Go2State, IMU, WebRtcReq

# Transform and time utilities
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import std_msgs.msg


class DimOSIntegrationBridge(Node):
    """
    Integration bridge between optimized Go2 SDK and DimOS framework.
    
    This bridge:
    1. Translates our optimized point clouds to DimOS spatial memory format
    2. Provides costmap data for DimOS navigation planning
    3. Bridges robot state information between systems
    4. Enables DimOS agent framework to use our optimizations
    """
    
    def __init__(self):
        super().__init__('dimos_integration_bridge')
        
        self.get_logger().info("Initializing DimOS Integration Bridge")
        
        # QoS profiles for different data types
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Transform listener for coordinate frame conversions
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # State tracking
        self.latest_robot_state: Optional[Go2State] = None
        self.latest_imu_data: Optional[IMU] = None
        self.latest_pointcloud: Optional[PointCloud2] = None
        self.spatial_memory_data: Dict[str, Any] = {}
        
        # Performance monitoring
        self.processing_stats = {
            'point_clouds_processed': 0,
            'costmaps_generated': 0,
            'transform_lookups': 0,
            'avg_processing_time': 0.0
        }
        
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_services()
        
        # Start monitoring thread
        self.monitor_thread = threading.Thread(target=self._monitor_performance, daemon=True)
        self.monitor_thread.start()
        
        self.get_logger().info("DimOS Integration Bridge initialized successfully")
    
    def _setup_subscribers(self):
        """Set up subscribers for data from optimized Go2 SDK components."""
        
        # Subscribe to optimized lidar output
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar/filtered',
            self._lidar_callback,
            self.sensor_qos
        )
        
        # Subscribe to robot state
        self.state_sub = self.create_subscription(
            Go2State,
            '/go2_states',
            self._robot_state_callback,
            self.sensor_qos
        )
        
        # Subscribe to IMU data
        self.imu_sub = self.create_subscription(
            IMU,
            '/imu',
            self._imu_callback,
            self.sensor_qos
        )
        
        # Subscribe to WebRTC commands for monitoring
        self.webrtc_sub = self.create_subscription(
            WebRtcReq,
            '/webrtc_req',
            self._webrtc_callback,
            self.reliable_qos
        )
        
        self.get_logger().info("Subscribers configured")
    
    def _setup_publishers(self):
        """Set up publishers for DimOS-compatible data."""
        
        # Costmap for DimOS navigation
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.reliable_qos
        )
        
        # Global map for DimOS spatial memory
        self.global_map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            self.reliable_qos
        )
        
        # Enhanced point cloud with metadata for DimOS
        self.enhanced_cloud_pub = self.create_publisher(
            PointCloud2,
            '/lidar/enhanced',
            self.sensor_qos
        )
        
        # Spatial memory updates
        self.spatial_memory_pub = self.create_publisher(
            std_msgs.msg.String,
            '/spatial_memory/updates',
            self.reliable_qos
        )
        
        self.get_logger().info("Publishers configured")
    
    def _setup_services(self):
        """Set up services for DimOS integration."""
        # Services can be added here for more advanced integration
        pass
    
    def _lidar_callback(self, msg: PointCloud2):
        """Process optimized lidar data for DimOS integration."""
        start_time = time.time()
        
        try:
            self.latest_pointcloud = msg
            
            # Convert point cloud to costmap for DimOS navigation
            costmap = self._pointcloud_to_costmap(msg)
            if costmap:
                self.costmap_pub.publish(costmap)
                self.processing_stats['costmaps_generated'] += 1
            
            # Create enhanced point cloud with metadata
            enhanced_cloud = self._enhance_pointcloud(msg)
            if enhanced_cloud:
                self.enhanced_cloud_pub.publish(enhanced_cloud)
            
            # Update spatial memory
            self._update_spatial_memory(msg)
            
            # Update performance stats
            processing_time = time.time() - start_time
            self.processing_stats['point_clouds_processed'] += 1
            self.processing_stats['avg_processing_time'] = (
                (self.processing_stats['avg_processing_time'] * 
                 (self.processing_stats['point_clouds_processed'] - 1) + processing_time) /
                self.processing_stats['point_clouds_processed']
            )
            
        except Exception as e:
            self.get_logger().error(f"Error processing lidar data: {e}")
    
    def _robot_state_callback(self, msg: Go2State):
        """Handle robot state updates for DimOS integration."""
        self.latest_robot_state = msg
        
        # Log important state changes
        if hasattr(self, '_last_mode') and self._last_mode != msg.mode:
            self.get_logger().info(f"Robot mode changed: {self._last_mode} -> {msg.mode}")
        self._last_mode = msg.mode
    
    def _imu_callback(self, msg: IMU):
        """Handle IMU data for DimOS integration."""
        self.latest_imu_data = msg
    
    def _webrtc_callback(self, msg: WebRtcReq):
        """Monitor WebRTC commands for integration logging."""
        self.get_logger().debug(f"WebRTC command: api_id={msg.api_id}, topic={msg.topic}")
    
    def _pointcloud_to_costmap(self, cloud_msg: PointCloud2) -> Optional[OccupancyGrid]:
        """
        Convert optimized point cloud to costmap for DimOS navigation.
        
        This leverages the performance optimizations from our C++ lidar processor
        while providing the costmap format expected by DimOS navigation systems.
        """
        try:
            # Create costmap header
            costmap = OccupancyGrid()
            costmap.header = Header()
            costmap.header.stamp = self.get_clock().now().to_msg()
            costmap.header.frame_id = "map"
            
            # Costmap parameters (adjust based on requirements)
            resolution = 0.05  # 5cm resolution
            width = 400  # 20m x 20m map
            height = 400
            origin_x = -10.0  # Center the map around robot
            origin_y = -10.0
            
            costmap.info.resolution = resolution
            costmap.info.width = width
            costmap.info.height = height
            costmap.info.origin.position.x = origin_x
            costmap.info.origin.position.y = origin_y
            costmap.info.origin.position.z = 0.0
            costmap.info.origin.orientation.w = 1.0
            
            # Initialize costmap data
            costmap.data = [0] * (width * height)
            
            # Process point cloud data (simplified - in practice would use optimized C++ processing)
            # This would normally interface with our optimized lidar processor's output
            
            # For now, create a simple obstacle map based on point density
            # In full integration, this would use the spatial hash and memory pool data
            
            return costmap
            
        except Exception as e:
            self.get_logger().error(f"Error creating costmap: {e}")
            return None
    
    def _enhance_pointcloud(self, cloud_msg: PointCloud2) -> Optional[PointCloud2]:
        """
        Enhance point cloud with additional metadata for DimOS spatial memory.
        
        Adds fields like:
        - Confidence scores from our optimized filtering
        - Temporal stability information
        - Spatial hash cell IDs
        """
        try:
            # Create enhanced cloud with additional fields
            enhanced_cloud = PointCloud2()
            enhanced_cloud.header = cloud_msg.header
            enhanced_cloud.height = cloud_msg.height
            enhanced_cloud.width = cloud_msg.width
            enhanced_cloud.is_bigendian = cloud_msg.is_bigendian
            enhanced_cloud.is_dense = cloud_msg.is_dense
            
            # Add metadata fields
            enhanced_cloud.fields = list(cloud_msg.fields)
            
            # Add confidence field
            confidence_field = PointField()
            confidence_field.name = "confidence"
            confidence_field.offset = enhanced_cloud.fields[-1].offset + 4
            confidence_field.datatype = PointField.FLOAT32
            confidence_field.count = 1
            enhanced_cloud.fields.append(confidence_field)
            
            # Add spatial hash cell ID field
            hash_field = PointField()
            hash_field.name = "spatial_hash_id"
            hash_field.offset = confidence_field.offset + 4
            hash_field.datatype = PointField.UINT32
            hash_field.count = 1
            enhanced_cloud.fields.append(hash_field)
            
            enhanced_cloud.point_step = hash_field.offset + 4
            enhanced_cloud.row_step = enhanced_cloud.point_step * enhanced_cloud.width
            
            # Copy and enhance data (simplified for now)
            enhanced_cloud.data = cloud_msg.data  # Would add metadata in full implementation
            
            return enhanced_cloud
            
        except Exception as e:
            self.get_logger().error(f"Error enhancing point cloud: {e}")
            return None
    
    def _update_spatial_memory(self, cloud_msg: PointCloud2):
        """Update DimOS spatial memory with optimized lidar data."""
        try:
            # Create spatial memory update
            memory_data = {
                'timestamp': time.time(),
                'frame_id': cloud_msg.header.frame_id,
                'point_count': cloud_msg.width * cloud_msg.height,
                'performance_optimized': True,
                'processing_stats': self.processing_stats.copy()
            }
            
            # Add robot pose if available
            if self.latest_robot_state:
                memory_data['robot_position'] = {
                    'x': self.latest_robot_state.position[0],
                    'y': self.latest_robot_state.position[1],
                    'z': self.latest_robot_state.position[2]
                }
            
            self.spatial_memory_data[str(time.time())] = memory_data
            
            # Publish spatial memory update
            import json
            memory_msg = std_msgs.msg.String()
            memory_msg.data = json.dumps(memory_data)
            self.spatial_memory_pub.publish(memory_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error updating spatial memory: {e}")
    
    def _monitor_performance(self):
        """Monitor and log performance metrics."""
        while rclpy.ok():
            try:
                time.sleep(10.0)  # Log every 10 seconds
                
                stats = self.processing_stats
                self.get_logger().info(
                    f"DimOS Bridge Performance - "
                    f"Point clouds: {stats['point_clouds_processed']}, "
                    f"Costmaps: {stats['costmaps_generated']}, "
                    f"Avg processing time: {stats['avg_processing_time']:.3f}s"
                )
                
            except Exception as e:
                self.get_logger().error(f"Error in performance monitoring: {e}")
    
    def get_integration_status(self) -> Dict[str, Any]:
        """Get current integration status for DimOS monitoring."""
        return {
            'bridge_active': True,
            'latest_data_timestamp': time.time(),
            'performance_stats': self.processing_stats.copy(),
            'robot_state_available': self.latest_robot_state is not None,
            'imu_data_available': self.latest_imu_data is not None,
            'pointcloud_available': self.latest_pointcloud is not None,
            'spatial_memory_entries': len(self.spatial_memory_data)
        }


def main(args=None):
    """Main entry point for DimOS integration bridge."""
    rclpy.init(args=args)
    
    try:
        bridge = DimOSIntegrationBridge()
        
        # Spin the node
        rclpy.spin(bridge)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in DimOS integration bridge: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()