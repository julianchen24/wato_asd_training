#include "costmap_core.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

}

void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
