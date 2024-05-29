#ifndef _CLOUD_HELPERS_HPP_
#define _CLOUD_HELPERS_HPP_

#include <fstream>
#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace cloud_helpers {

void readParamsFromFile(const std::string &parameters_file_name,
                        int &filtering_meank, double &filtering_stddevmulthresh,
                        double &voxelization_leaf_size_x,
                        double &voxelization_leaf_size_y,
                        double &voxelization_leaf_size_z,
                        unsigned &segmentation_k_search_count,
                        unsigned &segmentation_min_cluster_size,
                        unsigned &segmentation_max_cluster_size,
                        unsigned &segmentation_num_neighbors,
                        double &segmentation_smoothness_threshold,
                        double &segmentation_curvature_threshold) {
  std::ifstream params_file(parameters_file_name);
  std::string param_name;

  params_file >> param_name >> filtering_meank;
  params_file >> param_name >> filtering_stddevmulthresh;
  params_file >> param_name >> voxelization_leaf_size_x;
  params_file >> param_name >> voxelization_leaf_size_y;
  params_file >> param_name >> voxelization_leaf_size_z;
  params_file >> param_name >> segmentation_k_search_count;
  params_file >> param_name >> segmentation_min_cluster_size;
  params_file >> param_name >> segmentation_max_cluster_size;
  params_file >> param_name >> segmentation_num_neighbors;
  params_file >> param_name >> segmentation_smoothness_threshold;
  params_file >> param_name >> segmentation_curvature_threshold;

  params_file.close();

  std::cout << "Parameters Used:" << std::endl;
  std::cout << "filtering_meank: " << filtering_meank << std::endl; //{30};
  std::cout << "filtering_stddevmulthresh: " << filtering_stddevmulthresh
            << std::endl; //{1.0f};
  std::cout << "voxelization_leaf_size_x: " << voxelization_leaf_size_x
            << std::endl; //{0.3f};
  std::cout << "voxelization_leaf_size_y: " << voxelization_leaf_size_y
            << std::endl; //{0.3f};
  std::cout << "voxelization_leaf_size_z: " << voxelization_leaf_size_z
            << std::endl; //{0.3f};
  std::cout << "segmentation_k_search_count: " << segmentation_k_search_count
            << std::endl; //{50};
  std::cout << "segmentation_min_cluster_size: "
            << segmentation_min_cluster_size << std::endl; //{300};
  std::cout << "segmentation_max_cluster_size: "
            << segmentation_max_cluster_size << std::endl; //{1000000};
  std::cout << "segmentation_num_neighbors: " << segmentation_num_neighbors
            << std::endl; //{30};
  std::cout << "segmentation_smoothness_threshold: "
            << segmentation_smoothness_threshold
            << std::endl; //{3.0 / 180.0 * M_PI};
  std::cout << "segmentation_curvature_threshold: "
            << segmentation_curvature_threshold << std::endl; //{1.0};
}

void printCloudInfo(const std::string &cloud_name,
                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  std::cout << cloud_name << " has: " << cloud->width * cloud->height
            << " data points." << std::endl;
}

void printCloudPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  for (const auto &point : *cloud)
    std::cout << "    " << point.x << " " << point.y << " " << point.z
              << std::endl;
}

void printSegmentedCloudInfo(
    const std::string &cloud_name,
    const std::vector<pcl::PointIndices> &clusters,
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
  std::cout << cloud_name << " has: " << cloud->width * cloud->height
            << " data points." << std::endl;
  std::cout << cloud_name << " has: " << clusters.size() << " clusters."
            << std::endl;
}

void writeCloudToFile(std::string file_name,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
  pcl::io::savePCDFileASCII(file_name, *cloud_ptr);
}

// Returns -1 if loading failed, 0 otherwise
int loadCloudFromFile(std::string file_name,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud_ptr) ==
      -1) //* load the file
  {
    std::string error_message = "Couldn't read file " + file_name + "\n";
    PCL_ERROR(error_message.c_str());
    return -1;
  }
  std::cout << "Loaded " << cloud_ptr->width * cloud_ptr->height
            << " data points from " << file_name
            << " with the following fields: " << std::endl;

  return 0;
}

} // namespace cloud_helpers

#endif /* _CLOUD_HELPERS_HPP_ */
