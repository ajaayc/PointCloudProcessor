#include <iostream>

#include "point_cloud_subscriber_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


int main(int argc, char **argv) {
  std::cout << "hello world ros_pcl_visualizer package\n";
  std::cout << "Reached here\n";

   std::string parameters_file_name = "parameters.txt";


  int filtering_meank;                      //{30};
  double filtering_stddevmulthresh;         //{1.0f};
  double voxelization_leaf_size_x;          //{0.3f};
  double voxelization_leaf_size_y;          //{0.3f};
  double voxelization_leaf_size_z;          //{0.3f};
  unsigned segmentation_k_search_count;     //{50};
  unsigned segmentation_min_cluster_size;   //{300};
  unsigned segmentation_max_cluster_size;   //{1000000};
  unsigned segmentation_num_neighbors;      //{30};
  double segmentation_smoothness_threshold; //{3.0 / 180.0 * M_PI};
  double segmentation_curvature_threshold;  //{1.0};

  cloud_helpers::readParamsFromFile(
      parameters_file_name, filtering_meank, filtering_stddevmulthresh,
      voxelization_leaf_size_x, voxelization_leaf_size_y,
      voxelization_leaf_size_z, segmentation_k_search_count,
      segmentation_min_cluster_size, segmentation_max_cluster_size,
      segmentation_num_neighbors, segmentation_smoothness_threshold,
      segmentation_curvature_threshold);

  // const rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudSubscriberNode>(
      filtering_meank, filtering_stddevmulthresh, voxelization_leaf_size_x,
      voxelization_leaf_size_y, voxelization_leaf_size_z,
      segmentation_k_search_count, segmentation_min_cluster_size,
      segmentation_max_cluster_size, segmentation_num_neighbors,
      segmentation_smoothness_threshold, segmentation_curvature_threshold));
  rclcpp::shutdown();
  return 0;

  return 0;
}
