#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap",10);
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  initializeCostmap();
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
      double angle = scan->angle_min + i * scan->angle_increment;
      double range = scan->ranges[i];
      if (range < scan->range_max && range > scan->range_min) {
          // Calculate grid coordinates
          int x_grid, y_grid;
          convertToGrid(range, angle, x_grid, y_grid);
          markObstacle(x_grid, y_grid);
      }
  }

}

void CostmapNode::initializeCostmap() {
  cost_map_.info.resolution = 0.1;
  cost_map_.info.width = 300;
  cost_map_.info.height = 300;
  cost_map_.info.origin.position.x = -15;
  cost_map_.info.origin.position.y = -15;
  cost_map_.info.origin.position.z = 0;
  cost_map_grid_.assign(cost_map_.info.width,std::vector<int>(cost_map_.info.height,0));
}

void CostmapNode::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
  // Convert to cartesian coordinates
  double x = range*cos(angle); 
  double y = range*sin(angle);
  x_grid = static_cast<int>((x - cost_map_.info.origin.position.x)/cost_map_.info.resolution);
  y_grid = static_cast<int>((y - cost_map_.info.origin.position.y)/cost_map_.info.resolution);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid <cost_map_.info.width && y_grid >= 0 && y_grid < cost_map.info.height) {
    cost_map_grid_[x_grid][y_grid] = 100;
  }
}

void CostmapNode::inflateObstacles(){
  const int inflation_radius = 1;
  const double inflation_grid_radius = inflation_radius/cost_map_.info.resolution;
  const int max_cost = 100;

  for (int i = 0; i < cost_map_.info.width; i++) {
    for (int j = 0; j < cost_map_.info.height; j++) {
      if (cost_map_grid_[i][j] == max_cost) {
        for (int dx = i - inflation_grid_radius; dx <= i + inflation_grid_radius; dx++) {
          for (int dy = j - inflation_grid_radius; dy <= j + inflation_grid_radius; dy++) {
            if (dx >= 0 && dx < cost_map_.info.width &&
                dy >= 0 && dy < cost_map_.info.height) {
              double distance = std::sqrt(
                std::pow((dx - i) * cost_map_.info.resolution, 2) +
                std::pow((dy - j) * cost_map_.info.resolution, 2));
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
  cost_map_.data.resize(cost_map_.info.width * cost_map_.info.height, -1);
  for (int j = 0; j < cost_map_.info.height; j++) {
    for (int i = 0; i < cost_map_.info.width; i++) {
      int index = j * cost_map_.info.width + i;
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