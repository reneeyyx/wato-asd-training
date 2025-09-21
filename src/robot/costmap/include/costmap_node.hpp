#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "costmap_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>

 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void initializeCostmap();
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap();
 
  private:
    int width_;
    int height_;
    float resolution_;
    float inflation_radius_;
    robot::CostmapCore costmap_;
    

    // Place these constructs here
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    nav_msgs::msg::OccupancyGrid cost_map_;
    std::vector<std::vector<int>> cost_map_grid_;

};
 
#endif 