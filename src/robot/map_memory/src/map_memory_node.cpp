#include "map_memory_node.hpp"
#include <cmath>
#include <functional>
#include <chrono>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {

  // Subscribe to the costmap topic
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "costmap", rclcpp::QoS(1).transient_local(),
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );

  // Subscribe to the odometry topic
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );

  // Publisher for the map memory
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).reliable().transient_local());

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MapMemoryNode::updateMap, this)
  );

  initializeGlobalMap();

}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);

  has_odom_ = true;

  if (!map_initialized_) {
    global_map_.header.frame_id = "sim_world";
    global_map_.header.stamp = this->now();
    map_pub_->publish(global_map_);
    map_initialized_ = true;
  }
}

void MapMemoryNode::updateMap(){
  if (costmap_updated_ && has_odom_ && map_initialized_){
    integrateCostmap();
    global_map_.header.frame_id = "sim_world";
    global_map_.header.stamp = this->now();
    map_pub_->publish(global_map_);
    // reset the flag
    costmap_updated_ = false;
  }
}

void MapMemoryNode::integrateCostmap(){

  if (!has_odom_) return;

  const int costmap_width = static_cast<int>(latest_costmap_.info.width);
  const int costmap_height = static_cast<int>(latest_costmap_.info.height);
  const double costmap_resolution = latest_costmap_.info.resolution;
  const double global_resolution = global_map_.info.resolution;

  if (std::abs(costmap_resolution - global_resolution) > 1e-6) return;

  for (int cy{0}; cy<costmap_height; ++cy){
    for (int cx{0}; cx<costmap_width; ++cx){
      const int costmap_index = cy * costmap_width + cx;
      const int8_t costmap_value = latest_costmap_.data[costmap_index];
      if (costmap_value == -1) continue; // unknown cell, skip

      const double local_x = latest_costmap_.info.origin.position.x + (cx) * costmap_resolution;
      const double local_y = latest_costmap_.info.origin.position.y + (cy) * costmap_resolution;

      const double cost_yaw = std::cos(robot_yaw_);
      const double cost_xaw = std::sin(robot_yaw_);
      const double obstacle_world_x = robot_x_ + (local_x * cost_yaw - local_y * cost_xaw);
      const double obstacle_world_y = robot_y_ + (local_x * cost_xaw + local_y * cost_yaw);

      rayTrace(robot_x_, robot_y_, obstacle_world_x, obstacle_world_y, costmap_value);
    }
  }
}

void MapMemoryNode::rayTrace(double start_x, double start_y, double end_x, double end_y, int8_t obstacle_value){
  const double global_origin_x = global_map_.info.origin.position.x;
  const double global_origin_y = global_map_.info.origin.position.y;
  const double global_resolution = global_map_.info.resolution;
  const int global_width = static_cast<int>(global_map_.info.width);
  const int global_height = static_cast<int>(global_map_.info.height);

  const double dx = end_x - start_x;
  const double dy = end_y - start_y;
  const double distance = std::hypot(dx, dy);

  if (distance < 1e-6) return;

  const double step_size = global_resolution / 2.0;
  const int steps = static_cast<int>(distance / step_size);
  const double step_x = dx / steps;
  const double step_y = dy / steps;

  for (int i{0}; i<steps;++i){
    const double current_x = start_x + i * step_x;
    const double current_y = start_y + i * step_y;

    const int gx = static_cast<int>((current_x - global_origin_x) / global_resolution);
    const int gy = static_cast<int>((current_y - global_origin_y) / global_resolution);

    if (gx>=0 && gx<global_width && gy>=0 && gy<global_height){
      const int global_index = gy*global_width + gx;

      if (global_map_.data[global_index] == -1){
        global_map_.data[global_index] = 0; // free space
      }
    }
  }

  const int obstacle_gx = static_cast<int>((end_x - global_origin_x) / global_resolution);
  const int obstacle_gy = static_cast<int>((end_y - global_origin_y) / global_resolution);

  if (obstacle_gx>=0 && obstacle_gx < global_width && obstacle_gy > 0 && obstacle_gy < global_height){
    const int obstacle_index = obstacle_gy * global_width + obstacle_gx;

    if (global_map_.data[obstacle_index] != 100){
      global_map_.data[obstacle_index] = obstacle_value;
    }
  }
}

void MapMemoryNode::initializeGlobalMap(){
  const double resolution = 0.1;
  const unsigned int width = 600;
  const unsigned int height = 600;

  global_map_.header.frame_id = "sim_world";
  global_map_.header.stamp = this->now();
  global_map_.info.resolution = resolution;
  global_map_.info.width = width;
  global_map_.info.height = height;
  global_map_.info.origin.position.x = -30.0;
  global_map_.info.origin.position.y = -30.0;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;

  global_map_.data.assign(width * height, -1); // unknown
}

double MapMemoryNode::quaternionToYaw(const geometry_msgs::msg::Quaternion& quat){
  const double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  const double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
