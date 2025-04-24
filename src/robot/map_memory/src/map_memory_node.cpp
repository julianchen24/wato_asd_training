#include "map_memory_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",10);

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

  global_map_.header.frame_id = "map";

  global_map_.info.resolution = 0.1; 
  global_map_.info.width = 100;       
  global_map_.info.height = 100;    
  global_map_.info.origin.position.x = -25.0;
  global_map_.info.origin.position.y = -25.0;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;

  global_map_.data.resize(global_map_.info.width * global_map_.info.height, -1);
  map_pub_->publish(global_map_);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
  if (distance >= distance_threshold_) {
    last_x = x;
    last_y = y;
    should_update_map_ = true;
    latest_odom_ = *msg;
  }

}

void MapMemoryNode::updateMap() {
  if (costmap_updated_) {
    integrateCostmap();
    map_pub_->publish(global_map_);
    should_update_map_ = false;

  }

}

void MapMemoryNode::integrateCostmap() {

  double robot_x = latest_odom_.pose.pose.position.x;
  double robot_y = latest_odom_.pose.pose.position.y;

  const geometry_msgs::msg::Quaternion &q = latest_odom_.pose.pose.orientation;
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 mat(quat);
  double roll,pitch,yaw;
  mat.getRPY(roll,pitch,yaw);
  
  for (size_t j{0}; j < latest_costmap_.info.height; ++j) {
    for (size_t i{0}; i < latest_costmap_.info.width; ++i) {
      int local_index = j * latest_costmap_.info.width + i;

      int8_t cost = latest_costmap_.data[local_index];
      if (cost == -1 || cost == 0) {
        continue;
      }
      double local_x = (i * latest_costmap_.info.resolution) + latest_costmap_.info.origin.position.x;
      double local_y = (j * latest_costmap_.info.resolution) + latest_costmap_.info.origin.position.y;

      double global_x = robot_x + local_x * std::cos(yaw) - local_y * std::sin(yaw);
      double global_y = robot_y + local_x * std::sin(yaw) + local_y * std::cos(yaw);

      int global_i = static_cast<int>((global_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
      int global_j = static_cast<int>((global_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

      if (global_i < 0 || global_i >= global_map_.info.width || global_j < 0 || global_j >= global_map_.info.height) {
        continue;
      }

      int global_index = global_j * global_map_.info.width + global_i;

      if (global_map_.data[global_index] == -1 || cost > global_map_.data[global_index]) {
        global_map_.data[global_index] = cost;
      }
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
