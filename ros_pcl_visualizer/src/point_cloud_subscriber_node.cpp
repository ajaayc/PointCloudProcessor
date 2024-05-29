#include <iostream>

#include "cloud_helpers.hpp"
#include "point_cloud_visualization_manager.hpp"
#include "point_cloud_processor_engine.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "point_cloud_subscriber_node.hpp"

int main(int argc, char ** argv)
{
  std::cout << "hello world ros_pcl_visualizer package\n";
  std::cout << "Reached here\n";

  //const rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudSubscriberNode>());
  rclcpp::shutdown();
  return 0;

  return 0;
}
