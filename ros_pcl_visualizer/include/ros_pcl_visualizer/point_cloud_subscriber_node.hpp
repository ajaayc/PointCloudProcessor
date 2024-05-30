#ifndef _POINT_CLOUD_SUBSCRIBER_NODE_HPP_
#define _POINT_CLOUD_SUBSCRIBER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "cloud_helpers.hpp"
#include "point_cloud_processor_engine.hpp"
#include "point_cloud_visualization_manager.hpp"

using std::placeholders::_1;

class PointCloudSubscriberNode : public rclcpp::Node {
public:
  PointCloudSubscriberNode(
      int filtering_meank,                      //{30};
      double filtering_stddevmulthresh,         //{1.0f};
      double voxelization_leaf_size_x,          //{0.3f};
      double voxelization_leaf_size_y,          //{0.3f};
      double voxelization_leaf_size_z,          //{0.3f};
      unsigned segmentation_k_search_count,     //{50};
      unsigned segmentation_min_cluster_size,   //{300};
      unsigned segmentation_max_cluster_size,   //{1000000};
      unsigned segmentation_num_neighbors,      //{30};
      double segmentation_smoothness_threshold, //{3.0 / 180.0 * M_PI};
      double segmentation_curvature_threshold,  //{1.0};
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("point_cloud_subscriber", options),
        input_topic_name{"/tof_camera/xyz"},
        output_topic_name{"/processed_cloud"},
        eng{filtering_meank,                   //{30};
            filtering_stddevmulthresh,         //{1.0f};
            voxelization_leaf_size_x,          //{0.3f};
            voxelization_leaf_size_y,          //{0.3f};
            voxelization_leaf_size_z,          //{0.3f};
            segmentation_k_search_count,       //{50};
            segmentation_min_cluster_size,     //{300};
            segmentation_max_cluster_size,     //{1000000};
            segmentation_num_neighbors,        //{30};
            segmentation_smoothness_threshold, //{3.0 / 180.0 * M_PI};
            segmentation_curvature_threshold}  //{1.0};} {
  {
    publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_name, 2);
    subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_name, 10,
        std::bind(&PointCloudSubscriberNode::pointCloudTopicCallback, this,
                  _1));
  }

protected:
  void
  pointCloudTopicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    std::cout << "Entering callback fn" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());

    // ROS2 Pointcloud2 to PCL Pointcloud2
    pcl_conversions::moveToPCL(*msg, *cloud2);

    // PCL Pointcloud2 to PCL Pointcloud
    pcl::fromPCLPointCloud2(*cloud2, *cloud1);

    eng.setOriginalCloud(cloud1);

    cloud_helpers::printCloudInfo(eng.getOriginalCloudName(),
                                  eng.getOriginalCloud());

    eng.filterOriginalCloud();

    cloud_helpers::printCloudInfo(eng.getFilteredCloudName(),
                                  eng.getFilteredCloud());

    eng.voxelizeFilteredCloud();

    cloud_helpers::printCloudInfo(eng.getVoxelizedCloudName(),
                                  eng.getVoxelizedCloud());

    eng.segmentVoxelizedCloud();

    cloud_helpers::printSegmentedCloudInfo(eng.getSegmentedCloudName(),
                                           eng.getSegmentedCloudClusters(),
                                           eng.getSegmentedCloud());

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr segmented_cloud =
        eng.getSegmentedCloud();

    // cloud_helpers::printCloudPoints(segmented_cloud);

    pcl::PCLPointCloud2::Ptr segmented_cloud2(new pcl::PCLPointCloud2());

    pcl::toPCLPointCloud2(*segmented_cloud, *segmented_cloud2);

    // std::cout << "segmented_cloud2 width: " << segmented_cloud2->width
    //           << std::endl;
    // std::cout << "segmented_cloud2 height: " << segmented_cloud2->height
    //           << std::endl;

    // Convert to ros message
    sensor_msgs::msg::PointCloud2 cloud_out;
    pcl_conversions::moveFromPCL(*segmented_cloud2, cloud_out);

    // std::cout << "cloud_out.width: " << cloud_out.width << std::endl;

    // std::cout << "cloud_out.height: " << cloud_out.height << std::endl;

    // std::cout << "cloud_out.is_dense: " << cloud_out.is_dense << std::endl;

    // For debugging
    // cloud_out = *msg;
    publisher->publish(cloud_out);
    return;
  }

  // ROS 2 subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber;
  std::string input_topic_name;
  // ROS 2 publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
  std::string output_topic_name;

  PointCloudProcessorEngine eng;
};

#endif // _POINT_CLOUD_SUBSCRIBER_NODE_HPP_