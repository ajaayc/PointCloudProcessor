#ifndef _POINT_CLOUD_SUBSCRIBER_NODE_HPP_
#define _POINT_CLOUD_SUBSCRIBER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;

class PointCloudSubscriberNode : public rclcpp::Node {
public:
  PointCloudSubscriberNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("point_cloud_subscriber", options),
        input_topic_name{"/tof_camera/xyz"}, output_topic_name{
                                                 "/processed_cloud"} {
    publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_name, 2);
    subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_name, 10,
        std::bind(&PointCloudSubscriberNode::pointCloudTopicCallback, this,
                  _1));
  }

protected:
  void
  pointCloudTopicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    sensor_msgs::msg::PointCloud2 cloud_out;
    cloud_out = *msg;
    publisher->publish(cloud_out);
    return;
  }

  // ROS 2 subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber;
  std::string input_topic_name;
  // ROS 2 publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
  std::string output_topic_name;
};

#endif // _POINT_CLOUD_SUBSCRIBER_NODE_HPP_