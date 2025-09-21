#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    // robot::MapMemoryCore map_memory_;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void updateMap();
  void integrateCostmap();
  void initializeGlobalMap();
  void rayTrace(double start_x, double start_y, double end_x, double end_y, int8_t obstacle_value);
  double quaternionToYaw(const geometry_msgs::msg::Quaternion& quat);

  robot::MapMemoryCore map_memory_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid latest_costmap_;
  nav_msgs::msg::OccupancyGrid global_map_;

  bool costmap_updated_ = false;
  bool has_odom_ = false;
  bool map_initialized_ = false;

  double robot_x_ = 0.0, robot_y_ = 0.0, robot_yaw_ = 0.0;
};

#endif 
