#include "control_node.hpp"
#include <chrono>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  lookahead_distance_ = 1.0;
  goal_tolerance_ = 0.1;
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

  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  auto cmd_vel = computeVelocity(*lookahead_point);
  cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  // Implement
  auto robot_position = this->robot_odom_->pose.pose.position;

  for(const auto& pose_stamped : this->current_path_->poses) {
    if (this->computeDistance(robot_position, pose_stamped.pose.position) >= this->lookahead_distance_) {
      return pose_stamped;
    }
  }

  if (this->computeDistance(robot_position, this->current_path_->poses.back().pose.position) < this->goal_tolerance_) {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0; 
    stop.angular.z = 0.0;
    this->cmd_vel_pub_->publish(stop);
    return std::nullopt;
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
