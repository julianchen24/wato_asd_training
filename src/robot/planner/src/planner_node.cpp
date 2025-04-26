#include "planner_node.hpp"
PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())), 
                             state_(State::WAITING_FOR_GOAL) {
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  
  // Publishers
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path",10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (msg->data.empty() || msg->info.width == 0 || msg->info.height == 0) {
    RCLCPP_WARN(this->get_logger(), "Received empty or invalid map");
    return;
  }

  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL && goal_received_) {
    planPath();
  }

}
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  if (goal_.header.frame_id.empty()) {
    goal_.header.frame_id = current_map_.header.frame_id;
  }
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;

  if (!current_map_.data.empty()) {
    planPath();
  }
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (current_map_.data.empty() || !goal_received_) {
      RCLCPP_INFO(this->get_logger(), "Waiting for required data...");
      return;
    }

    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
      nav_msgs::msg::Path empty_path;
      empty_path.header.stamp = this->now();
      empty_path.header.frame_id = "sim_world";
      path_pub_->publish(empty_path);
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.2;
  
}

void PlannerNode::planPath() {
  RCLCPP_INFO(this->get_logger(), "Calling planPath()");
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }
  

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "sim_world";

  CellIndex start = worldToGrid(robot_pose_.position.x, robot_pose_.position.y);
  CellIndex goal = worldToGrid(goal_.point.x, goal_.point.y);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  open.push(AStarNode(start, 0.0));

  std::unordered_set<CellIndex, CellIndexHash> closed;

  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  g_score[start] = 0.0;

  while (!open.empty()) {
    AStarNode top = open.top();
    open.pop();

    CellIndex current = top.index;
    if (closed.count(current)) {
      continue;
    }
    closed.insert(current);

    // Path has been found
    if (current == goal) {
      std::vector<CellIndex> cell_path;
      while (current != start) {
        cell_path.push_back(current);
        current = came_from[current];
      }
      cell_path.push_back(start);
      std::reverse(cell_path.begin(), cell_path.end());

      if (!cell_path.empty()) {
        for (const auto& cell : cell_path) {
          geometry_msgs::msg::PoseStamped pose;
          pose.header = path.header;
          pose.pose.position.x = cell.x * current_map_.info.resolution + current_map_.info.origin.position.x + 0.5 * current_map_.info.resolution;
          pose.pose.position.y = cell.y * current_map_.info.resolution + current_map_.info.origin.position.y + 0.5 * current_map_.info.resolution;
          pose.pose.position.z = 0.0; 
          pose.pose.orientation.w = 1.0;
          path.poses.push_back(pose);
        }
      }
      if (!path.poses.empty()) {
        RCLCPP_INFO(this->get_logger(), "Publishing path with %zu points", path.poses.size());
        path_pub_->publish(path);
        return;
      }
      
    }

    std::vector<CellIndex> neighbors = {
      {current.x + 1, current.y}, {current.x - 1, current.y},
      {current.x, current.y + 1}, {current.x, current.y - 1},
      {current.x + 1, current.y + 1}, {current.x + 1, current.y - 1},
      {current.x - 1, current.y + 1}, {current.x - 1, current.y - 1}
    };

    for (const CellIndex& neighbor : neighbors) {
      if (neighbor.x < 0 || neighbor.y < 0 || neighbor.x >= static_cast<int>(current_map_.info.width) || neighbor.y >= static_cast<int>(current_map_.info.height)) {
        continue;
      }

      int grid_index = neighbor.y * current_map_.info.width + neighbor.x;
      if (grid_index >= static_cast<int>(current_map_.data.size())) {
        continue;
      }

      if (current_map_.data[grid_index] > 50) {
        continue;
      }

      if (closed.count(neighbor)) {
        continue;
      }

      double tentative_g = g_score[current] + std::hypot(neighbor.x - current.x, neighbor.y - current.y);

      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        g_score[neighbor] = tentative_g;
        double h = std::hypot(goal.x - neighbor.x, goal.y - neighbor.y);
        double f = tentative_g + h;
        open.push(AStarNode(neighbor, f));
        came_from[neighbor] = current;

        
      }
    }
  }

  if (path.poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "No path found");
      // Publish empty path to clear any previous path
      return;
  }
  path_pub_->publish(path);
}

CellIndex PlannerNode::worldToGrid(double x, double y) const {
  // int gx = static_cast<int>(std::round((x - current_map_.info.origin.position.x) / current_map_.info.resolution));
  // int gy = static_cast<int>(std::round((y - current_map_.info.origin.position.y) / current_map_.info.resolution));

  // gx = std::max(0, std::min(gx, static_cast<int>(current_map_.info.width - 1)));
  // gy = std::max(0, std::min(gy, static_cast<int>(current_map_.info.height - 1)));

  const auto &info = current_map_.info;
  double fx = (x - info.origin.position.x) / info.resolution;
  double fy = (y - info.origin.position.y) / info.resolution;

  int gx = static_cast<int>(std::floor(fx));
  int gy = static_cast<int>(std::floor(fy));

  gx = std::clamp(gx, 0, static_cast<int>(info.width)  - 1);
  gy = std::clamp(gy, 0, static_cast<int>(info.height) - 1);
  return CellIndex(gx, gy);
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
