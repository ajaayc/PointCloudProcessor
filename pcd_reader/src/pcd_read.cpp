#include <fstream>
#include <iostream>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <thread>

#include "cloud_helpers.hpp"
#include "point_cloud_visualization_manager.hpp"
#include "point_cloud_processor_engine.hpp"

int main() {
  std::string original_cloud_file_name = "../scans.pcd";
  std::string parameters_file_name = "../parameters.txt";

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

  pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud_ptr{new pcl::PointCloud<pcl::PointXYZ>};

  if (cloud_helpers::loadCloudFromFile(original_cloud_file_name, original_cloud_ptr) == -1) {
    return -1;
  }

  PointCloudProcessorEngine eng{filtering_meank,
                                filtering_stddevmulthresh,
                                voxelization_leaf_size_x,
                                voxelization_leaf_size_y,
                                voxelization_leaf_size_z,
                                segmentation_k_search_count,
                                segmentation_min_cluster_size,
                                segmentation_max_cluster_size,
                                segmentation_num_neighbors,
                                segmentation_smoothness_threshold,
                                segmentation_curvature_threshold};

  eng.setOriginalCloud(original_cloud_ptr);

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

  std::string original_cloud_name = eng.getOriginalCloudName();
  std::string filtered_cloud_name = eng.getFilteredCloudName();
  std::string voxelized_cloud_name = eng.getVoxelizedCloudName();
  std::string segmented_cloud_name = eng.getSegmentedCloudName();

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr original_cloud =
      eng.getOriginalCloud();
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr filtered_cloud =
      eng.getFilteredCloud();
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr voxelized_cloud =
      eng.getVoxelizedCloud();
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr segmented_cloud =
      eng.getSegmentedCloud();

  PointCloudVisualizationManager visualization_manager;

  visualization_manager.addOriginalCloud(original_cloud_name, original_cloud);
  visualization_manager.addFilteredCloud(filtered_cloud_name, filtered_cloud);
  visualization_manager.addVoxelizedCloud(voxelized_cloud_name,
                                          voxelized_cloud);
  visualization_manager.addSegmentedCloud(
      segmented_cloud_name, eng.getSegmentedCloudClusters().size(),
      segmented_cloud);

  visualization_manager.runVisualization();

  return 0;
}
