#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
 
#include "costmap_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  
  // Initialize the constructs and their parameters
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}


void CostmapNode::initalizeCostMap() {
  double map_width = 10.0;
  double map_height = 10.0;
  x_grid = static_cast<int>(map_width / resolution);
  y_grid = static_cast<int>(map_height / resolution);
  OccupancyGrid.resize(x_grid, std::vector<int>(y_grid,0));
}


// This might be problematic for the future, return if necessary
void CostmapNode::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
  x_grid = range*std::cos(angle);
  y_grid = range*std::sin(angle);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
  OccupancyGrid[x_grid][y_grid] = 100;

}

void CostmapNode::inflateObstacles() {
  double radius = 1.0;
  double max_cost = 100.0;
  double cell_radius = static_cast<int>(radius / resolution);

  for (int y{0}; y < y_grid; ++y) {
    for (int x{0}; x < x_grid; ++x) {
      if (OccupancyGrid[x][y] == most) {
        for (int dy{cell_radius}; dy >= -cell_radius; --dy) {
          for (int dx{cell_radius}; dx >= -cell_radius; --dx) {
            int i = x + dx;
            int j = y + dy;
            if (i < 0 || i >= x_grid || j < 0 || j >= y_grid) {
              continue;
            }
            if (OccupancyGrid[i][j] == max_cost) {
              continue;
            }
            double distance = std::sqrt(std::pow(dy,2) + std::pow(dx,2));
            double temp_cost = max_cost * (1 - (distance/radius));
            if (temp_cost > OccupancyGrid[i][j]) {
              OccupancyGrid[i][j] = temp_cost;
            }
          }
        }  
      }
    }
  }
}

void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid costmap_msg;
  costmap_msg.header.frame_id = "map";
  costmap_msg.header.stamp = this->now();
  costmap_msg.info.resolution = resolution;
  costmap_msg.info.width = x_grid;
  costmap_msg.info.height = y_grid;
  costmap_msg.info.origin.position.x = -(x_grid * resolution)/2;
  costmap_msg.info.origin.position.y = -(y_grid * resolution)/2;

  std::vector<int8_t> data(x_grid * y_grid, 0);
  for (int y{0}; y < y_grid; ++y) {
    for (int x{0}; x < x_grid; ++x) {
      data.push_back(static_cast<int8_t>(OccupancyGrid[x][y]));
    }
  }
  costmap_msg.data = data;
  costmap_pub_->publish(costmap_msg);
}


void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  initalizeCostMap();

  for (size_t i{0}; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
      int x_grid, y_grid;
      convertToGrid(range,angle,x_grid,y_grid);
      markObstacle(x_grid,y_grid);
    }
  }
  inflateObstacles();
  publishCostmap();
}

 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
