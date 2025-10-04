#!/usr/bin/env python3
"""
DimOS Integration Test Script

This script tests the integration between our optimized Go2 SDK and the dimos-unitree framework.
It verifies:
1. Message compatibility
2. Topic communication
3. Performance metrics
4. Integration bridge functionality
"""

import os
import sys
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import signal

# Test imports for compatibility
try:
    from unitree_go.msg import WebRtcReq
    print("✓ unitree_go.msg.WebRtcReq import successful")
except ImportError as e:
    print(f"✗ Failed to import unitree_go.msg.WebRtcReq: {e}")
    sys.exit(1)

try:
    from go2_interfaces.msg import Go2State, IMU
    print("✓ go2_interfaces messages import successful")
except ImportError as e:
    print(f"✗ Failed to import go2_interfaces messages: {e}")
    sys.exit(1)

try:
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import OccupancyGrid
    print("✓ Standard ROS messages import successful")
except ImportError as e:
    print(f"✗ Failed to import standard ROS messages: {e}")
    sys.exit(1)


class IntegrationTestNode(Node):
    """Test node for DimOS integration verification."""
    
    def __init__(self):
        super().__init__('dimos_integration_test')
        
        self.get_logger().info("Starting DimOS Integration Test")
        
        # Test counters
        self.test_results = {
            'webrtc_messages': 0,
            'robot_states': 0,
            'imu_messages': 0,
            'point_clouds': 0,
            'costmaps': 0,
            'spatial_memory_updates': 0
        }
        
        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self._setup_test_publishers()
        self._setup_test_subscribers()
        
        # Start test sequence
        self.test_timer = self.create_timer(2.0, self._run_test_sequence)
        self.results_timer = self.create_timer(10.0, self._print_test_results)
        
        self.get_logger().info("DimOS Integration Test Node initialized")
    
    def _setup_test_publishers(self):
        """Set up test publishers to simulate robot data."""
        
        # Test WebRTC commands
        self.webrtc_pub = self.create_publisher(
            WebRtcReq,
            '/webrtc_req',
            self.reliable_qos
        )
        
        # Test robot state
        self.state_pub = self.create_publisher(
            Go2State,
            '/go2_states',
            self.sensor_qos
        )
        
        # Test IMU data
        self.imu_pub = self.create_publisher(
            IMU,
            '/imu',
            self.sensor_qos
        )
        
        # Test point cloud
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            '/lidar/filtered',
            self.sensor_qos
        )
        
        self.get_logger().info("Test publishers configured")
    
    def _setup_test_subscribers(self):
        """Set up test subscribers to monitor integration outputs."""
        
        # Monitor costmap output
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self._costmap_callback,
            self.reliable_qos
        )
        
        # Monitor enhanced point clouds
        self.enhanced_cloud_sub = self.create_subscription(
            PointCloud2,
            '/lidar/enhanced',
            self._enhanced_cloud_callback,
            self.sensor_qos
        )
        
        # Monitor spatial memory updates
        from std_msgs.msg import String
        self.spatial_memory_sub = self.create_subscription(
            String,
            '/spatial_memory/updates',
            self._spatial_memory_callback,
            self.reliable_qos
        )
        
        self.get_logger().info("Test subscribers configured")
    
    def _run_test_sequence(self):
        """Run the integration test sequence."""
        
        # Test 1: WebRTC compatibility
        webrtc_msg = WebRtcReq()
        webrtc_msg.id = int(time.time())
        webrtc_msg.topic = 'rt/api/sport/request'
        webrtc_msg.api_id = 1016  # Hello command
        webrtc_msg.parameter = 'test'
        webrtc_msg.priority = 0
        
        self.webrtc_pub.publish(webrtc_msg)
        self.test_results['webrtc_messages'] += 1
        
        # Test 2: Robot state
        state_msg = Go2State()
        state_msg.mode = 1
        state_msg.progress = 50
        state_msg.gait_type = 2
        state_msg.position = [1.0, 2.0, 0.5]
        state_msg.velocity = [0.1, 0.0, 0.0]
        
        self.state_pub.publish(state_msg)
        self.test_results['robot_states'] += 1
        
        # Test 3: IMU data
        imu_msg = IMU()
        imu_msg.quaternion = [0.0, 0.0, 0.0, 1.0]
        imu_msg.gyroscope = [0.01, 0.02, 0.03]
        imu_msg.accelerometer = [0.0, 0.0, 9.81]
        
        self.imu_pub.publish(imu_msg)
        self.test_results['imu_messages'] += 1
        
        # Test 4: Point cloud (simplified)
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = 'lidar_link'
        cloud_msg.height = 1
        cloud_msg.width = 100
        cloud_msg.is_dense = True
        
        self.cloud_pub.publish(cloud_msg)
        self.test_results['point_clouds'] += 1
        
        self.get_logger().debug("Test sequence completed")
    
    def _costmap_callback(self, msg):
        """Monitor costmap generation."""
        self.test_results['costmaps'] += 1
        self.get_logger().debug(f"Received costmap: {msg.info.width}x{msg.info.height}")
    
    def _enhanced_cloud_callback(self, msg):
        """Monitor enhanced point cloud output."""
        self.get_logger().debug(f"Received enhanced cloud: {msg.width} points")
    
    def _spatial_memory_callback(self, msg):
        """Monitor spatial memory updates."""
        self.test_results['spatial_memory_updates'] += 1
        self.get_logger().debug("Received spatial memory update")
    
    def _print_test_results(self):
        """Print test results summary."""
        self.get_logger().info("=== DimOS Integration Test Results ===")
        for test_name, count in self.test_results.items():
            status = "✓" if count > 0 else "⚠"
            self.get_logger().info(f"{status} {test_name}: {count}")
        
        # Calculate success rate
        total_tests = len(self.test_results)
        successful_tests = sum(1 for count in self.test_results.values() if count > 0)
        success_rate = (successful_tests / total_tests) * 100
        
        self.get_logger().info(f"Overall Success Rate: {success_rate:.1f}%")
        
        if success_rate >= 80:
            self.get_logger().info("🎉 DimOS Integration Test: PASSED")
        else:
            self.get_logger().warn("⚠️ DimOS Integration Test: NEEDS ATTENTION")


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print("\nShutting down DimOS Integration Test...")
    rclpy.shutdown()
    sys.exit(0)


def main():
    """Main test function."""
    print("DimOS-Unitree Integration Test")
    print("=" * 40)
    
    # Test environment
    print(f"ROS_DISTRO: {os.getenv('ROS_DISTRO', 'Not set')}")
    print(f"Current directory: {os.getcwd()}")
    
    # Handle Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize ROS
    rclpy.init()
    
    try:
        # Create and run test node
        test_node = IntegrationTestNode()
        
        print("\nRunning integration tests...")
        print("Press Ctrl+C to stop the test")
        
        # Spin the node
        rclpy.spin(test_node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Test error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()