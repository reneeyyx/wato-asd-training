#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "control_core.hpp"

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path::SharedPtr latest_path_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    void controlLoop();
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped & lookahead_point);
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    double extractYaw(const geometry_msgs::msg::Quaternion &q);
    bool isGoalReached();
    
};

#endif
