#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
 
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishMessage();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void initalizeCostMap(); 
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap(); 
 
  private:
    robot::CostmapCore costmap_;
    std::vector<std::vector<int>> OccupancyGrid;
    double resolution = 0.1;
    int x_grid, y_grid;
    // Place these constructs here
    rclcpp::TimerBase::SharedPtr timer_;
    // For your “Hello, ROS 2!” test
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
   // Costmap publisher (you need this!)
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

};
 
#endif 