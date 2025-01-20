#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"
#include "costmap_node.hpp"
using namespace std;

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  std::vector<std::vector<int>> costmap_;
  double resolution_ = 0.1;
  int w = 300;
  int h = 300;
  int max_ = 100;
  costmap_.resize(h_, std::vector<int>(w_, 0));
  subscribe_Lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::callbackFuncLaser, this, std::placeholders::_1));
  publish_costmap_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}
void callbackFuncLaser(const sensor_msgs::msg::LaserScan::SharedPtr s)
{
  int array[300][300] = {0};
  for (size_t i = 0; i < s->ranges.size(); ++i)
  {
    if (s->ranges[i] < s->range_max && s->ranges[i] > s->range_min)
    {
      double angle = s->angle_min + i * s->angle_increment;
      double range = s->ranges[i];

      // Convert to Cartesian coordinates
      double x = s->ranges[i] * cos(angle);
      double y = s->ranges[i] * sin(angle);

      int xgrid = static_cast<int>(round(x / resolution_));
      int ygrid = static_cast<int>(round(y / resolution_));

      if (xgrid >= 0 && xgrid < w_ && ygrid >= 0 && ygrid < h_)
      {
        costmap_[grid_y][grid_x] = max_;
      }
    }
  }
}
void CostmapNode::inflateObstacles()
{
  for (int x = 0; x < h_; x++)
  {
    for (int y = 0; y < w_; y++)
    {
      if (costmap_[x][y] == max)
      {
        for (int dx = -inflation_radius / resolution_; dx <= inflation_radius / resolution_; dx++)
        {
          for (int dy = -inflation_radius / resolution_; dy <= inflation_radius / resolution_; dy++)
          {
            int nx = x + dx;
            int ny = y + dy;
            if ((x + dx) >= 0 && (x + dx) < w_ && (y + dy) >= 0 && (y + dy) < h_)
            {
              double d = std::sqrt(dx * dx + dy * dy) / 10;
              if (d <= inflationRadius)
              {
                int inflated_cost = static_cast<int>(max_ * (1.0 - d / inflation_radius));
                costmap_[x + dx][y + dy] = std::max(costmap_[ny][nx], inflated_cost);
              }
            }
          }
        }
      }
    }
  }
}

// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}