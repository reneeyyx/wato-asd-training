#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"

struct CellIndex {
  int x;
  int y;
  CellIndex(int xx, int yy);
  CellIndex();
  bool operator==(const CellIndex &other) const;
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const;
};

// Structure representing a node in the A* open set
struct AStarNode {
  CellIndex index;
  double f_score;  // f = g + h
  AStarNode(CellIndex idx, double f);
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b);
};

// ------------------- Planner Node -------------------

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();

private:
  // State machine
  enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
  State state_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data
  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PointStamped goal_;
  geometry_msgs::msg::Pose robot_pose_;

  bool goal_received_;
  bool map_received_;

  // Planning parameters
  double goal_tolerance_ = 0.5;          // meters to consider goal reached
  double obstacle_threshold_ = 50.0;     // occupancy >= threshold => obstacle
  rclcpp::Time last_plan_time_;
  rclcpp::Duration replan_timeout_ = rclcpp::Duration::from_seconds(10.0);

  // -------------------- Callbacks --------------------
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  // -------------------- Helpers --------------------
  bool goalReached();
  std::optional<CellIndex> worldToGrid(double wx, double wy);
  geometry_msgs::msg::Point gridToWorld(const CellIndex &ci);
  bool isCellFree(const CellIndex &ci);
  void planPath();
};

#endif 
