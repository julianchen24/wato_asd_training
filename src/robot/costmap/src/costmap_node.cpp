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
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}
 
// Define the timer to publish a message every 500ms
// void CostmapNode::publishMessage() {
//   auto message = std_msgs::msg::String();
//   message.data = "Hello, ROS 2!";
//   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//   string_pub_->publish(message);
// }


void CostmapNode::initalizeCostMap() {
  double map_width = 100.0;
  double map_height = 100.0;
  x_grid = static_cast<int>(map_width / resolution);
  y_grid = static_cast<int>(map_height / resolution);
  if (OccupancyGrid.empty()) {
    OccupancyGrid.resize(x_grid, std::vector<int>(y_grid, 0));
  } else {
    for (auto& row : OccupancyGrid) {
      std::fill(row.begin(), row.end(), 0);
    }
  }
}


void CostmapNode::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
  // double x_world = range * std::cos(angle);
  // double y_world = range * std::sin(angle);

  // x_grid = static_cast<int>((x_world / resolution) + (this->x_grid / 2));
  // y_grid = static_cast<int>((y_world / resolution) + (this->y_grid / 2));

  double x_local = range * std::cos(angle);
  double y_local = range * std::sin(angle);

  double map_w = static_cast<double>(this->x_grid) * resolution;
  double map_h = static_cast<double>(this->y_grid) * resolution;

  double x_world = x_local + map_w * 0.5;
  double y_world = y_local + map_h * 0.5;

  double fx = x_world / resolution;
  double fy = y_world / resolution;

  int ix = static_cast<int>(std::floor(fx));
  int iy = static_cast<int>(std::floor(fy));
  ix = std::clamp(ix, 0, this->x_grid - 1);
  iy = std::clamp(iy, 0, this->y_grid - 1);

  // **use the parameter names** here**
  x_grid = ix;
  y_grid = iy;
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
  OccupancyGrid[x_grid][y_grid] = 100;

}

void CostmapNode::inflateObstacles() {
  double radius = 1.5; 
  double max_cost = 100.0;
  int cell_radius = static_cast<int>(radius / resolution);
  auto temp_grid = OccupancyGrid;
  
  for (int y = 0; y < y_grid; ++y) {
    for (int x = 0; x < x_grid; ++x) {
      if (OccupancyGrid[x][y] == 100) {
        for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
          for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
            int i = x + dx;
            int j = y + dy;
            if (i < 0 || i >= x_grid || j < 0 || j >= y_grid) {
              continue;
            }
            
            if (temp_grid[i][j] == 100) {
              continue;
            }
            
            double distance = std::sqrt(dx*dx + dy*dy) * resolution;
            if (distance <= radius) {
              int cost = static_cast<int>(max_cost * (1.0 - (distance / radius)));
              if (cost > temp_grid[i][j]) {
                temp_grid[i][j] = cost;
              }
            }
          }
        }
      }
    }
  }
  OccupancyGrid = temp_grid;
}

void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid costmap_msg;
  costmap_msg.header.frame_id = "sim_world";
  costmap_msg.header.stamp = this->now();
  costmap_msg.info.resolution = resolution;
  costmap_msg.info.width = x_grid;
  costmap_msg.info.height = y_grid;
  costmap_msg.info.origin.position.x = -(x_grid * resolution)/2;
  costmap_msg.info.origin.position.y = -(y_grid * resolution)/2;

  std::vector<int8_t> data(x_grid * y_grid, 0);
  for (int y{0}; y < y_grid; ++y) {
    for (int x{0}; x < x_grid; ++x) {
      data[y * x_grid + x] = static_cast<int8_t>(OccupancyGrid[x][y]);
    }
  }
  costmap_msg.data = data;
  costmap_pub_->publish(costmap_msg);
}


void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  OccupancyGrid.clear();
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
