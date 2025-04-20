#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
 
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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
#endif 