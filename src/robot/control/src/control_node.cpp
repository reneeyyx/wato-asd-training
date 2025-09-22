#include "control_node.hpp"


ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  lookahead_distance_ = 1.0;
  goal_tolerance_ = 0.1;
  linear_speed_ = 0.5;

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {latest_path_ = msg;});

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {latest_odom_ = msg;});
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),[this]() { controlLoop(); });
  
}

void ControlNode::controlLoop(){
  // Skip control if no path or odometry data is available
  if (!latest_path_ || !latest_odom_) {
      return;
  }

  // Check if goal is reached
  if (isGoalReached()) {
      // Stop the robot
      geometry_msgs::msg::Twist stop_cmd;
      stop_cmd.linear.x = 0.0;
      stop_cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(stop_cmd);
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      return;
  }

  // Find the lookahead point
  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
      return;  // No valid lookahead point found
  }

  // Compute velocity command
  auto cmd_vel = computeVelocity(*lookahead_point);

  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  // Add missing null checks
  if (!latest_path_ || latest_path_->poses.empty()) {
      return std::nullopt;
  }

  const auto& current_pos = latest_odom_->pose.pose.position;
  
  // Find the closest point on the path first
  size_t closest_idx = 0;
  double min_distance = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < latest_path_->poses.size(); ++i) {
      double distance = computeDistance(latest_path_->poses[i].pose.position, current_pos);
      if (distance < min_distance) {
          min_distance = distance;
          closest_idx = i;
      }
  }
  
  // Search forward from the closest point for the lookahead point
  for (size_t i = closest_idx; i < latest_path_->poses.size(); ++i) {
      double distance = computeDistance(latest_path_->poses[i].pose.position, current_pos);
      
      if (distance >= lookahead_distance_) {
          return latest_path_->poses[i];
      }
  }
  
  // If no point found at lookahead distance, return the last point (goal)
  // This handles the case when approaching the end of the path
  return latest_path_->poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped & lookahead_point) {
    geometry_msgs::msg::Twist cmd_vel;
    
    // Calculate the angle to the lookahead point
    const auto& current_pos = latest_odom_->pose.pose.position;
    const auto& target_pos = lookahead_point.pose.position;
    
    double dx = target_pos.x - current_pos.x;
    double dy = target_pos.y - current_pos.y;
    double target_yaw = std::atan2(dy, dx);
    
    // Get current orientation
    double current_yaw = extractYaw(latest_odom_->pose.pose.orientation);
    
    // Calculate angular error
    double yaw_error = target_yaw - current_yaw;
    
    // Normalize angle to [-pi, pi]
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
    
    // Set velocities
    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = 2.0 * linear_speed_ * std::sin(yaw_error) / lookahead_distance_; // Pure pursuit formula
    
    return cmd_vel;
}

bool ControlNode::isGoalReached() {
    if (!latest_path_ || latest_path_->poses.empty()) {
        return false;
    }
    
    const auto& current_pos = latest_odom_->pose.pose.position;
    const auto& goal_pos = latest_path_->poses.back().pose.position;
    
    double distance_to_goal = computeDistance(current_pos, goal_pos);
    return distance_to_goal <= goal_tolerance_;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
