#include "planner_node.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <limits>

CellIndex::CellIndex(int xx, int yy) : x(xx), y(yy) {}
CellIndex::CellIndex() : x(0), y(0) {}
bool CellIndex::operator==(const CellIndex &other) const {
  return (x == other.x && y == other.y);
}

std::size_t CellIndexHash::operator()(const CellIndex &idx) const {
  std::size_t h1 = std::hash<int>()(idx.x);
  std::size_t h2 = std::hash<int>()(idx.y);
  return h1 ^ (h2 << 1);
}

AStarNode::AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}

bool CompareF::operator()(const AStarNode &a, const AStarNode &b) {
  return a.f_score > b.f_score;
}

// ------------------- PlannerNode Implementation -------------------

PlannerNode::PlannerNode()
: Node("planner_node"),
  state_(State::WAITING_FOR_GOAL),
  goal_received_(false),
  map_received_(false),
  last_plan_time_(this->now())
{
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 20, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer
  using namespace std::chrono_literals;
  timer_ = this->create_wall_timer(500ms, std::bind(&PlannerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "PlannerNode started");
}

// -------------------- Callbacks --------------------

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  map_received_ = true;
  RCLCPP_INFO(this->get_logger(), "Map received: size %u x %u",
              current_map_.info.width, current_map_.info.height);

  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL && goal_received_) {
    RCLCPP_INFO(this->get_logger(), "Map updated - replanning");
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)", goal_.point.x, goal_.point.y);

  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (!goal_received_) return;

  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached. Switching to WAITING_FOR_GOAL state.");
      state_ = State::WAITING_FOR_GOAL;
      goal_received_ = false;
      return;
    }

    rclcpp::Time now = this->now();
    if ((now - last_plan_time_) >= replan_timeout_) {
      RCLCPP_INFO(this->get_logger(), "Replan timeout -> replanning");
      planPath();
    }
  }
}

// -------------------- Helpers --------------------

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  double dist = std::sqrt(dx*dx + dy*dy);
  return dist < goal_tolerance_;
}

std::optional<CellIndex> PlannerNode::worldToGrid(double wx, double wy) {
  if (!map_received_) return std::nullopt;
  double ox = current_map_.info.origin.position.x;
  double oy = current_map_.info.origin.position.y;
  double res = current_map_.info.resolution;
  int ix = static_cast<int>(std::floor((wx - ox) / res));
  int iy = static_cast<int>(std::floor((wy - oy) / res));
  if (ix < 0 || iy < 0 ||
      ix >= static_cast<int>(current_map_.info.width) ||
      iy >= static_cast<int>(current_map_.info.height)) {
    return std::nullopt;
  }
  return CellIndex(ix, iy);
}

geometry_msgs::msg::Point PlannerNode::gridToWorld(const CellIndex &ci) {
  geometry_msgs::msg::Point p;
  double ox = current_map_.info.origin.position.x;
  double oy = current_map_.info.origin.position.y;
  double res = current_map_.info.resolution;
  p.x = ox + (ci.x + 0.5) * res;
  p.y = oy + (ci.y + 0.5) * res;
  p.z = 0.0;
  return p;
}

bool PlannerNode::isCellFree(const CellIndex &ci) {
  int w = static_cast<int>(current_map_.info.width);
  int h = static_cast<int>(current_map_.info.height);
  if (ci.x < 0 || ci.y < 0 || ci.x >= w || ci.y >= h) return false;
  int idx = ci.y * w + ci.x;
  int8_t val = current_map_.data[idx];
  if (val < 0) return true; // unknown -> free
  return (val < obstacle_threshold_);
}

void PlannerNode::planPath() {
  if (!goal_received_) {
    RCLCPP_WARN(this->get_logger(), "No goal to plan to.");
    return;
  }
  if (!map_received_) {
    RCLCPP_WARN(this->get_logger(), "No map available for planning.");
    return;
  }

  auto start_opt = worldToGrid(robot_pose_.position.x, robot_pose_.position.y);
  auto goal_opt  = worldToGrid(goal_.point.x, goal_.point.y);

  if (!start_opt.has_value()) {
    RCLCPP_WARN(this->get_logger(), "Start is out of map bounds.");
    return;
  }
  if (!goal_opt.has_value()) {
    RCLCPP_WARN(this->get_logger(), "Goal is out of map bounds.");
    return;
  }
  CellIndex start = start_opt.value();
  CellIndex goal = goal_opt.value();

  if (!isCellFree(start)) {
    RCLCPP_WARN(this->get_logger(), "Start cell is an obstacle.");
    return;
  }
  if (!isCellFree(goal)) {
    RCLCPP_WARN(this->get_logger(), "Goal cell is an obstacle.");
    return;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_queue;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, bool, CellIndexHash> closed;

  auto heuristic = [&](const CellIndex &a, const CellIndex &b) {
    double dx = double(a.x - b.x);
    double dy = double(a.y - b.y);
    return std::sqrt(dx*dx + dy*dy);
  };

  g_score[start] = 0.0;
  double f0 = heuristic(start, goal);
  open_queue.emplace(start, f0);

  int width = static_cast<int>(current_map_.info.width);
  int height = static_cast<int>(current_map_.info.height);

  const std::vector<std::pair<int,int>> neighbors = {
    {1,0},{-1,0},{0,1},{0,-1},
    {1,1},{1,-1},{-1,1},{-1,-1}
  };

  bool found = false;
  CellIndex found_node;

  while (!open_queue.empty()) {
    AStarNode top = open_queue.top();
    open_queue.pop();
    CellIndex current = top.index;

    if (closed[current]) continue;
    closed[current] = true;

    if (current == goal) {
      found = true;
      found_node = current;
      break;
    }

    for (auto &off : neighbors) {
      CellIndex nbr(current.x + off.first, current.y + off.second);

      if (nbr.x < 0 || nbr.y < 0 || nbr.x >= width || nbr.y >= height) continue;
      if (!isCellFree(nbr)) continue;

      double move_cost = (off.first != 0 && off.second != 0) ? std::sqrt(2.0) : 1.0;
      double tentative_g = g_score[current] + move_cost;

      if (!g_score.count(nbr) || tentative_g < g_score[nbr]) {
        g_score[nbr] = tentative_g;
        came_from[nbr] = current;
        double f = tentative_g + heuristic(nbr, goal);
        open_queue.emplace(nbr, f);
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(this->get_logger(), "A*: failed to find a path.");
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->get_clock()->now();
    empty_path.header.frame_id = current_map_.header.frame_id;
    path_pub_->publish(empty_path);
    last_plan_time_ = this->now();
    return;
  }

  std::vector<CellIndex> cell_path;
  CellIndex cur = found_node;
  cell_path.push_back(cur);
  while (!(cur == start)) {
    cur = came_from[cur];
    cell_path.push_back(cur);
  }
  std::reverse(cell_path.begin(), cell_path.end());

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->get_clock()->now();
  path_msg.header.frame_id = current_map_.header.frame_id;

  for (const auto &ci : cell_path) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path_msg.header;
    auto p = gridToWorld(ci);
    ps.pose.position = p;
    ps.pose.orientation.w = 1.0;
    path_msg.poses.push_back(ps);
  }

  path_pub_->publish(path_msg);
  last_plan_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", path_msg.poses.size());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
