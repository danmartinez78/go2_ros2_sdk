// Copyright (c) 2024, RoboVerse community
// SPDX-License-Identifier: BSD-3-Clause

#ifndef LIDAR_PROCESSOR_CPP__LIDAR_TO_POINTCLOUD_NODE_HPP_
#define LIDAR_PROCESSOR_CPP__LIDAR_TO_POINTCLOUD_NODE_HPP_

#include <memory>
#include <vector>
#include <string>
#include <unordered_set>
#include <mutex>
#include <atomic>
#include <thread>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/ply_io.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"

namespace lidar_processor_cpp
{

struct LidarConfig
{
  std::vector<std::string> robot_ip_list;
  std::string map_name;
  bool save_map;
  double save_interval;  // seconds
  int max_points;        // Maximum points to keep in memory
  double voxel_size;     // Voxel downsampling size
};

struct Point3D
{
  float x, y, z;
  
  Point3D(float x_val, float y_val, float z_val) 
    : x(x_val), y(y_val), z(z_val) {}
  
  bool operator==(const Point3D& other) const 
  {
    return std::abs(x - other.x) < 1e-6 && 
           std::abs(y - other.y) < 1e-6 && 
           std::abs(z - other.z) < 1e-6;
  }
};

struct Point3DHash
{
  std::size_t operator()(const Point3D& p) const 
  {
    // Optimized hash function for better distribution
    // Use bit mixing for better hash distribution
    uint32_t x_bits = *reinterpret_cast<const uint32_t*>(&p.x);
    uint32_t y_bits = *reinterpret_cast<const uint32_t*>(&p.y);
    uint32_t z_bits = *reinterpret_cast<const uint32_t*>(&p.z);
    
    uint64_t hash = (uint64_t(x_bits) << 32) | y_bits;
    hash ^= z_bits + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    
    return static_cast<std::size_t>(hash);
  }
};

// Memory pool for PCL point clouds to reduce allocations
class PointCloudPool
{
public:
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
  
  PointCloudPool(size_t initial_size = 10);
  ~PointCloudPool() = default;
  
  PointCloudPtr acquire();
  void release(PointCloudPtr cloud);
  size_t getPoolSize() const;
  
private:
  std::queue<PointCloudPtr> available_clouds_;
  mutable std::mutex pool_mutex_;
  std::atomic<size_t> total_created_{0};
  
  PointCloudPtr createNewCloud();
};

class PointCloudAggregator
{
public:
  explicit PointCloudAggregator(const LidarConfig& config);
  
  void addPoints(const std::vector<Point3D>& new_points);
  std::vector<Point3D> getPointsCopy() const;
  bool hasChanges() const;
  void markSaved();
  int getPointCount() const;
  
  // Performance optimizations
  void reserveCapacity(size_t capacity);
  void addPointsOptimized(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

private:
  LidarConfig config_;
  std::unordered_set<Point3D, Point3DHash> points_;
  mutable std::mutex points_mutex_;
  std::atomic<bool> points_changed_;
  
  // Performance optimization: track insertion order for efficient cleanup
  std::vector<Point3D> insertion_order_;
  size_t next_cleanup_index_{0};
  
  void performEfficientCleanup();
  Point3D roundPoint(const Point3D& point) const noexcept;
};

class LidarToPointCloudNode : public rclcpp::Node
{
public:
  LidarToPointCloudNode();

private:
  void declareParameters();
  LidarConfig loadConfiguration();
  void setupSubscriptions();
  void setupPublishers();
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishAggregatedPointcloud(const std_msgs::msg::Header& header);
  void saveMapCallback();
  void logConfiguration();

  LidarConfig config_;
  std::unique_ptr<PointCloudAggregator> aggregator_;
  std::unique_ptr<PointCloudPool> point_cloud_pool_;
  
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::TimerBase::SharedPtr save_timer_;
};

}  // namespace lidar_processor_cpp

#endif  // LIDAR_PROCESSOR_CPP__LIDAR_TO_POINTCLOUD_NODE_HPP_