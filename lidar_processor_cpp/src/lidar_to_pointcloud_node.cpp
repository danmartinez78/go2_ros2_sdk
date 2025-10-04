// Copyright (c) 2024, RoboVerse community
// SPDX-License-Identifier: BSD-3-Clause

#include "lidar_processor_cpp/lidar_to_pointcloud_node.hpp"
#include <algorithm>
#include <chrono>

namespace lidar_processor_cpp
{

// PointCloudPool implementation
PointCloudPool::PointCloudPool(size_t initial_size)
{
  for (size_t i = 0; i < initial_size; ++i) {
    available_clouds_.push(createNewCloud());
  }
}

PointCloudPool::PointCloudPtr PointCloudPool::acquire()
{
  std::lock_guard<std::mutex> lock(pool_mutex_);
  
  if (available_clouds_.empty()) {
    return createNewCloud();
  }
  
  auto cloud = available_clouds_.front();
  available_clouds_.pop();
  cloud->clear();  // Reset the cloud for reuse
  return cloud;
}

void PointCloudPool::release(PointCloudPtr cloud)
{
  if (!cloud) return;
  
  std::lock_guard<std::mutex> lock(pool_mutex_);
  
  // Only keep a reasonable number of clouds in the pool
  if (available_clouds_.size() < 20) {
    cloud->clear();
    available_clouds_.push(cloud);
  }
}

size_t PointCloudPool::getPoolSize() const
{
  std::lock_guard<std::mutex> lock(pool_mutex_);
  return available_clouds_.size();
}

PointCloudPool::PointCloudPtr PointCloudPool::createNewCloud()
{
  total_created_++;
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->reserve(10000);  // Pre-allocate space for better performance
  return cloud;
}

// PointCloudAggregator optimized implementation
PointCloudAggregator::PointCloudAggregator(const LidarConfig& config)
  : config_(config), points_changed_(false)
{
  // Pre-allocate capacity for better performance
  points_.reserve(config_.max_points * 1.2);  // 20% extra capacity
  insertion_order_.reserve(config_.max_points * 1.2);
}

void PointCloudAggregator::addPoints(const std::vector<Point3D>& new_points)
{
  std::lock_guard<std::mutex> lock(points_mutex_);
  
  // Batch process points for better performance
  for (const auto& point : new_points) {
    Point3D rounded_point = roundPoint(point);
    
    // Use insert result to avoid duplicate processing
    auto [it, inserted] = points_.insert(rounded_point);
    if (inserted) {
      insertion_order_.push_back(rounded_point);
    }
  }
  
  // Efficient cleanup when exceeding limit
  if (static_cast<int>(points_.size()) > config_.max_points) {
    performEfficientCleanup();
  }
  
  points_changed_ = true;
}

void PointCloudAggregator::addPointsOptimized(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  std::lock_guard<std::mutex> lock(points_mutex_);
  
  // Direct PCL cloud processing for better performance
  for (const auto& pcl_point : cloud->points) {
    if (!std::isfinite(pcl_point.x) || !std::isfinite(pcl_point.y) || !std::isfinite(pcl_point.z)) {
      continue;
    }
    
    Point3D rounded_point = roundPoint({pcl_point.x, pcl_point.y, pcl_point.z});
    
    auto [it, inserted] = points_.insert(rounded_point);
    if (inserted) {
      insertion_order_.push_back(rounded_point);
    }
  }
  
  if (static_cast<int>(points_.size()) > config_.max_points) {
    performEfficientCleanup();
  }
  
  points_changed_ = true;
}

Point3D PointCloudAggregator::roundPoint(const Point3D& point) const noexcept
{
  // Optimized rounding using bit manipulation
  constexpr float scale = 1000.0f;
  constexpr float inv_scale = 1.0f / scale;
  
  return Point3D(
    std::round(point.x * scale) * inv_scale,
    std::round(point.y * scale) * inv_scale,
    std::round(point.z * scale) * inv_scale
  );
}

void PointCloudAggregator::performEfficientCleanup()
{
  // Instead of expensive sorting, use FIFO cleanup
  size_t target_size = config_.max_points * 0.8;  // Keep 80% after cleanup
  size_t points_to_remove = points_.size() - target_size;
  
  for (size_t i = 0; i < points_to_remove && next_cleanup_index_ < insertion_order_.size(); ++i) {
    points_.erase(insertion_order_[next_cleanup_index_]);
    next_cleanup_index_++;
  }
  
  // Reset if we've processed most of the insertion order
  if (next_cleanup_index_ > insertion_order_.size() * 0.5) {
    // Rebuild insertion order with current points
    insertion_order_.clear();
    insertion_order_.reserve(points_.size());
    for (const auto& point : points_) {
      insertion_order_.push_back(point);
    }
    next_cleanup_index_ = 0;
  }
}

void PointCloudAggregator::reserveCapacity(size_t capacity)
{
  std::lock_guard<std::mutex> lock(points_mutex_);
  points_.reserve(capacity);
  insertion_order_.reserve(capacity);
}

std::vector<Point3D> PointCloudAggregator::getPointsCopy() const
{
  std::lock_guard<std::mutex> lock(points_mutex_);
  return std::vector<Point3D>(points_.begin(), points_.end());
}

bool PointCloudAggregator::hasChanges() const
{
  return points_changed_.load();
}

void PointCloudAggregator::markSaved()
{
  points_changed_ = false;
}

int PointCloudAggregator::getPointCount() const
{
  std::lock_guard<std::mutex> lock(points_mutex_);
  return static_cast<int>(points_.size());
}

LidarToPointCloudNode::LidarToPointCloudNode()
  : Node("lidar_to_pointcloud")
{
  // Declare and get parameters
  declareParameters();
  config_ = loadConfiguration();
  
  // Initialize components with performance optimizations
  aggregator_ = std::make_unique<PointCloudAggregator>(config_);
  point_cloud_pool_ = std::make_unique<PointCloudPool>(15);  // Pool of 15 clouds
  
  // Pre-allocate aggregator capacity
  aggregator_->reserveCapacity(config_.max_points);
  
  // Setup subscriptions and publishers
  setupSubscriptions();
  setupPublishers();
  
  // Setup timers
  if (config_.save_map) {
    save_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(config_.save_interval),
      std::bind(&LidarToPointCloudNode::saveMapCallback, this)
    );
  }
  
  // Log configuration
  logConfiguration();
  
  RCLCPP_INFO(this->get_logger(), "🚀 LiDAR Node initialized with performance optimizations");
}

void LidarToPointCloudNode::declareParameters()
{
  this->declare_parameter("robot_ip_lst", std::vector<std::string>{});
  this->declare_parameter("map_name", "3d_map");
  this->declare_parameter("map_save", "true");
  this->declare_parameter("save_interval", 10.0);
  this->declare_parameter("max_points", 1000000);
  this->declare_parameter("voxel_size", 0.01);
}

LidarConfig LidarToPointCloudNode::loadConfiguration()
{
  LidarConfig config;
  
  config.robot_ip_list = this->get_parameter("robot_ip_lst").as_string_array();
  config.map_name = this->get_parameter("map_name").as_string();
  std::string save_map_str = this->get_parameter("map_save").as_string();
  config.save_map = (save_map_str == "true");
  config.save_interval = this->get_parameter("save_interval").as_double();
  config.max_points = this->get_parameter("max_points").as_int();
  config.voxel_size = this->get_parameter("voxel_size").as_double();
  
  return config;
}

void LidarToPointCloudNode::setupSubscriptions()
{
  // Setup QoS profile for high-frequency data
  auto qos = rclcpp::QoS(1)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .history(rclcpp::HistoryPolicy::KeepLast);
  
  if (config_.robot_ip_list.size() == 1) {
    // Single robot mode
    auto subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/robot0/point_cloud2",
      qos,
      std::bind(&LidarToPointCloudNode::lidarCallback, this, std::placeholders::_1)
    );
    subscriptions_.push_back(subscription);
  } else {
    // Multi-robot mode
    for (size_t i = 0; i < config_.robot_ip_list.size(); ++i) {
      std::string topic = "/robot" + std::to_string(i) + "/point_cloud2";
      auto subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic,
        qos,
        std::bind(&LidarToPointCloudNode::lidarCallback, this, std::placeholders::_1)
      );
      subscriptions_.push_back(subscription);
    }
  }
}

void LidarToPointCloudNode::setupPublishers()
{
  auto qos = rclcpp::QoS(1)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .history(rclcpp::HistoryPolicy::KeepLast);
  
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pointcloud/aggregated", qos
  );
}

void LidarToPointCloudNode::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  try {
    // Use memory pool for better performance
    auto cloud = point_cloud_pool_->acquire();
    pcl::fromROSMsg(*msg, *cloud);
    
    // Use optimized direct PCL processing
    aggregator_->addPointsOptimized(cloud);
    
    // Return cloud to pool
    point_cloud_pool_->release(cloud);
    
    // Publish aggregated point cloud
    publishAggregatedPointcloud(msg->header);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing LiDAR data: %s", e.what());
  }
}

void LidarToPointCloudNode::publishAggregatedPointcloud(const std_msgs::msg::Header& header)
{
  try {
    auto points = aggregator_->getPointsCopy();
    if (points.empty()) {
      return;
    }
    
    // Use memory pool for publishing
    auto cloud = point_cloud_pool_->acquire();
    cloud->points.reserve(points.size());
    
    for (const auto& point : points) {
      cloud->points.emplace_back(point.x, point.y, point.z);
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    
    // Convert to ROS message
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(*cloud, pointcloud_msg);
    pointcloud_msg.header = header;
    
    pointcloud_pub_->publish(pointcloud_msg);
    
    // Return cloud to pool
    point_cloud_pool_->release(cloud);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error publishing point cloud: %s", e.what());
  }
}

void LidarToPointCloudNode::saveMapCallback()
{
  try {
    if (!aggregator_->hasChanges()) {
      return;
    }
    
    auto points = aggregator_->getPointsCopy();
    if (points.empty()) {
      return;
    }
    
    // Use memory pool for saving
    auto cloud = point_cloud_pool_->acquire();
    cloud->points.reserve(points.size());
    
    for (const auto& point : points) {
      cloud->points.emplace_back(point.x, point.y, point.z);
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    
    // Apply voxel downsampling for saving
    if (config_.voxel_size > 0) {
      auto downsampled_cloud = point_cloud_pool_->acquire();
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      voxel_filter.setInputCloud(cloud);
      voxel_filter.setLeafSize(config_.voxel_size, config_.voxel_size, config_.voxel_size);
      voxel_filter.filter(*downsampled_cloud);
      
      // Return original cloud to pool, use downsampled one
      point_cloud_pool_->release(cloud);
      cloud = downsampled_cloud;
    }
    
    // Save to file
    std::string map_filename = config_.map_name + ".ply";
    if (pcl::io::savePLYFileBinary(map_filename, *cloud) == 0) {
      int point_count = static_cast<int>(cloud->points.size());
      int total_points = aggregator_->getPointCount();
      aggregator_->markSaved();
      
      RCLCPP_INFO(this->get_logger(),
        "💾 Saved map: %s (%d downsampled / %d total points)",
        map_filename.c_str(), point_count, total_points);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save map: %s", map_filename.c_str());
    }
    
    // Return cloud to pool
    point_cloud_pool_->release(cloud);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error saving map: %s", e.what());
  }
}

void LidarToPointCloudNode::logConfiguration()
{
  RCLCPP_INFO(this->get_logger(), "🗺️  LiDAR Processor Configuration:");
  
  std::string robot_ips = "[";
  for (size_t i = 0; i < config_.robot_ip_list.size(); ++i) {
    robot_ips += config_.robot_ip_list[i];
    if (i < config_.robot_ip_list.size() - 1) robot_ips += ", ";
  }
  robot_ips += "]";
  
  RCLCPP_INFO(this->get_logger(), "   Robot IPs: %s", robot_ips.c_str());
  RCLCPP_INFO(this->get_logger(), "   Map name: %s", config_.map_name.c_str());
  RCLCPP_INFO(this->get_logger(), "   Save map: %s", config_.save_map ? "true" : "false");
  
  if (config_.save_map) {
    RCLCPP_INFO(this->get_logger(), "   Save interval: %.1fs", config_.save_interval);
    RCLCPP_INFO(this->get_logger(), "   Max points: %d", config_.max_points);
    RCLCPP_INFO(this->get_logger(), "   Voxel size: %.3fm", config_.voxel_size);
  }
}

}  // namespace lidar_processor_cpp

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<lidar_processor_cpp::LidarToPointCloudNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cerr << "Error running lidar processor: " << e.what() << std::endl;
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}