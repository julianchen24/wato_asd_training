#include "control_node.hpp"
#include <chrono>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  lookahead_distance_ = 1.0;  
  goal_tolerance_ = 0.2;      
  linear_speed_ = 0.5;       
  
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  double dist_to_goal = computeDistance(
    robot_odom_->pose.pose.position, 
    current_path_->poses.back().pose.position
  );

  if (dist_to_goal < goal_tolerance_) {
    RCLCPP_INFO(this->get_logger(), "Goal reached! Distance: %.2f", dist_to_goal);
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    RCLCPP_WARN(this->get_logger(), "No valid lookahead point found, stopping robot");
    return;
  }
  auto cmd_vel = computeVelocity(*lookahead_point);
  cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  // Implement
  if (!current_path_ || current_path_->poses.empty()) {
    return std::nullopt;
  }
  auto robot_position = this->robot_odom_->pose.pose.position;
  double distance_to_final = this->computeDistance(robot_position, this->current_path_->poses.back().pose.position);
  if (distance_to_final < this->goal_tolerance_) {
    return std::nullopt;
  }

  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < current_path_->poses.size(); i++) {
    double dist = this->computeDistance(robot_position, current_path_->poses[i].pose.position);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  for (size_t i = closest_idx; i < current_path_->poses.size(); i++) {
    double dist = this->computeDistance(robot_position, current_path_->poses[i].pose.position);
    if (dist >= this->lookahead_distance_) {
      RCLCPP_INFO(this->get_logger(), "Found lookahead point at index %zu", i);
      return current_path_->poses[i];
    }
  }

  return this->current_path_->poses.back();
}
geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  // Implement
  auto robot_position = robot_odom_->pose.pose.position;
  auto robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

  double angle = std::atan2(
    target.pose.position.y - robot_position.y,
    target.pose.position.x - robot_position.x
  );

  double alpha = angle - robot_yaw;

  while (alpha > M_PI) {
    alpha -= 2* M_PI;
  }
  while (alpha < -M_PI) {
    alpha += 2 * M_PI;
  }
  // To calculate the curvature of the circular arc to the lookahead point.
  // Comes from Pure Pursuit, curvature = 2 * sin(alpha) / lookahead_distance
  double curvature = 2.0 * std::sin(alpha) / lookahead_distance_;

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = linear_speed_ * curvature;
  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  // Implement
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
