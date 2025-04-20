#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

}

void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
