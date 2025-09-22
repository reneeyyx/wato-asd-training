#include <chrono>
#include <memory>
#include <cmath>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() 
  : Node("costmap"),
    width_(300),
    height_(300),
    resolution_(0.1),
    inflation_radius_(1.0),
    costmap_(robot::CostmapCore(this->get_logger())),
    robot_x_(0.0),
    robot_y_(0.0),
    robot_yaw_(0.0) {
  // Initialize the constructs and their parameters
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&CostmapNode::odomCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap",10);
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Don't process if we don't have robot pose yet
  if (robot_x_ == 0.0 && robot_y_ == 0.0 && robot_yaw_ == 0.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                         "No robot pose available yet, skipping costmap update");
    return;
  }
  
  initializeCostmap();
  
  int obstacle_count = 0;
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
      double angle = scan->angle_min + i * scan->angle_increment;
      double range = scan->ranges[i];
      if (range < scan->range_max && range > scan->range_min) {
          // Calculate grid coordinates
          int x_grid, y_grid;
          convertToGrid(range, angle, x_grid, y_grid);
          markObstacle(x_grid, y_grid);
          obstacle_count++;
      }
  }
  
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                       "Processed %d valid range measurements at pose (%.2f, %.2f, %.2f)", 
                       obstacle_count, robot_x_, robot_y_, robot_yaw_);
  
  inflateObstacles();
  publishCostmap();
}

void CostmapNode::initializeCostmap() {
  cost_map_.info.resolution = resolution_;
  cost_map_.info.width = width_;
  cost_map_.info.height = height_;
  cost_map_.info.origin.position.x = -15;
  cost_map_.info.origin.position.y = -15;
  cost_map_.info.origin.position.z = 0;
  cost_map_.header.frame_id = "sim_world";  // Use global frame
  cost_map_grid_.assign(height_,std::vector<int>(width_,0));
}

void CostmapNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  
  // Extract yaw from quaternion
  auto q = msg->pose.pose.orientation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  robot_yaw_ = std::atan2(siny_cosp, cosy_cosp);
}

void CostmapNode::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
  // Convert to cartesian coordinates in lidar frame
  double x_lidar = range * cos(angle); 
  double y_lidar = range * sin(angle);
  
  // Transform to global coordinates
  double x_global = robot_x_ + (x_lidar * cos(robot_yaw_) - y_lidar * sin(robot_yaw_));
  double y_global = robot_y_ + (x_lidar * sin(robot_yaw_) + y_lidar * cos(robot_yaw_));
  
  // Convert to grid coordinates
  x_grid = static_cast<int>((x_global - cost_map_.info.origin.position.x) / resolution_);
  y_grid = static_cast<int>((y_global - cost_map_.info.origin.position.y) / resolution_);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
    cost_map_grid_[x_grid][y_grid] = 100;
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Obstacle out of bounds: (%d, %d)", x_grid, y_grid);
  }
}

void CostmapNode::inflateObstacles(){
  const int inflation_radius = 1;
  const double inflation_grid_radius = inflation_radius/cost_map_.info.resolution;
  const int max_cost = 100;

  for (int i = 0; i < width_; i++) {
    for (int j = 0; j < height_; j++) {
      if (cost_map_grid_[i][j] == max_cost) {
        for (int dx = i - inflation_grid_radius; dx <= i + inflation_grid_radius; dx++) {
          for (int dy = j - inflation_grid_radius; dy <= j + inflation_grid_radius; dy++) {
            if (dx >= 0 && dx < width_ &&
                dy >= 0 && dy < height_) {
              double distance = std::sqrt(
                std::pow((dx - i) * resolution_, 2) +
                std::pow((dy - j) * resolution_, 2));
              if (distance <= inflation_radius) {
                int cost = static_cast<int>(max_cost * (1 - distance / inflation_radius));
                cost_map_grid_[dx][dy] = std::max(cost_map_grid_[dx][dy], cost);
              }
            }
          }
        }
      }
    }
  }
}

void CostmapNode::publishCostmap(){
  cost_map_.header.stamp = this->get_clock()->now();
  cost_map_.data.resize(width_ * height_);
  
  // Initialize all cells as free space (0), then mark obstacles
  for (int j = 0; j < height_; j++) {
    for (int i = 0; i < width_; i++) {
      int index = j * width_ + i;
      cost_map_.data[index] = cost_map_grid_[i][j];
    }
  }
  costmap_pub_->publish(cost_map_);
}

 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}