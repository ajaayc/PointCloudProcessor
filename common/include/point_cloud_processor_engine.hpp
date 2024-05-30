#ifndef _POINT_CLOUD_PROCESSOR_ENGINE_HPP_
#define _POINT_CLOUD_PROCESSOR_ENGINE_HPP_

#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>

class PointCloudProcessorEngine {
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud;
  std::vector<pcl::PointIndices> segmented_cloud_clusters;

  // Outlier filtering parameters
  const int filtering_meank;
  const double filtering_stddevmulthresh;

  // Voxelization parameters
  const double voxelization_leaf_size_x;
  const double voxelization_leaf_size_y;
  const double voxelization_leaf_size_z;

  // Segmentation parameters
  unsigned segmentation_k_search_count;
  unsigned segmentation_min_cluster_size;
  unsigned segmentation_max_cluster_size;
  unsigned segmentation_num_neighbors;
  double segmentation_smoothness_threshold;
  double segmentation_curvature_threshold;

public:
  PointCloudProcessorEngine(int filtering_meank,
                            double filtering_stddevmulthresh,
                            double voxelization_leaf_size_x,
                            double voxelization_leaf_size_y,
                            double voxelization_leaf_size_z,
                            unsigned segmentation_k_search_count,
                            unsigned segmentation_min_cluster_size,
                            unsigned segmentation_max_cluster_size,
                            unsigned segmentation_num_neighbors,
                            double segmentation_smoothness_threshold,
                            double segmentation_curvature_threshold)

      : original_cloud{new pcl::PointCloud<pcl::PointXYZ>},
        filtered_cloud{new pcl::PointCloud<pcl::PointXYZ>},
        voxelized_cloud{new pcl::PointCloud<pcl::PointXYZ>},
        segmented_cloud{new pcl::PointCloud<pcl::PointXYZRGB>},
        filtering_meank{filtering_meank},
        filtering_stddevmulthresh{filtering_stddevmulthresh},
        voxelization_leaf_size_x{voxelization_leaf_size_x},
        voxelization_leaf_size_y{voxelization_leaf_size_y},
        voxelization_leaf_size_z{voxelization_leaf_size_z},
        segmentation_k_search_count{segmentation_k_search_count},
        segmentation_min_cluster_size{segmentation_min_cluster_size},
        segmentation_max_cluster_size{segmentation_max_cluster_size},
        segmentation_num_neighbors{segmentation_num_neighbors},
        segmentation_smoothness_threshold{segmentation_smoothness_threshold},
        segmentation_curvature_threshold{segmentation_curvature_threshold} {}

  void filterOriginalCloud() { filterCloud(original_cloud, filtered_cloud); }

  void voxelizeFilteredCloud() {
    voxelizeCloud(filtered_cloud, voxelized_cloud);
  }

  void segmentVoxelizedCloud() {
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    segmentCloud(voxelized_cloud, reg, segmented_cloud_clusters);
    segmented_cloud = reg.getColoredCloud();
  }

  // If setOriginalCloud function is used, user is responsible for
  // rerunning, filterOriginalCloud, voxelizeFilteredCloud, and
  // segmentVoxelizedCloud (in that order).

  // NOTE: Ptr is copied, so if changes to original pointer are made
  // outside of the class, the changes will propagate to this object.
  void
  setOriginalCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr new_original_cloud) {
    original_cloud = new_original_cloud;
  }


  //Voxelizes the original point cloud, no filtering.
  void voxelizeOriginalCloud() {
    voxelizeCloud(original_cloud, voxelized_cloud);
  }

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr getOriginalCloud() const {
    return original_cloud;
  }

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr getFilteredCloud() const {
    return filtered_cloud;
  }

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr getVoxelizedCloud() const {
    return voxelized_cloud;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr getSegmentedCloud() const {
    return segmented_cloud;
  }

  const std::vector<pcl::PointIndices> &getSegmentedCloudClusters() const {
    return segmented_cloud_clusters;
  }

  std::string getOriginalCloudName() const { return "Original Cloud"; }

  std::string getFilteredCloudName() const { return "Filtered Cloud"; }

  std::string getVoxelizedCloudName() const { return "Voxelized Cloud"; }

  std::string getSegmentedCloudName() const { return "Segmented Cloud"; }


private:
  void voxelizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_cloud) {
#ifndef VOXELIZATION_DISABLED
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(input_cloud);
    voxel_grid.setLeafSize(voxelization_leaf_size_x, voxelization_leaf_size_y,
                           voxelization_leaf_size_z);
    voxel_grid.filter(*voxelized_cloud);
#else
    copyPointCloud(*input_cloud, *voxelized_cloud);
#endif
  }

  void filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud) {
    std::string filtered_cloud_filename{"filtered_cloud.pcd"};
#ifndef OBTAIN_FILTERED_RESULT_FROM_FILE
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(filtering_meank);
    sor.setStddevMulThresh(filtering_stddevmulthresh);
    sor.filter(*filtered_cloud);
    // Uncomment this line to enable writing the cloud to a file
    // cloud_helpers::writeCloudToFile(filtered_cloud_filename, filtered_cloud);
#else
    cloud_helpers::loadCloudFromFile(filtered_cloud_filename, filtered_cloud);
    // copyPointCloud(*input_cloud, *filtered_cloud);
#endif

    return;
  }

  void segmentCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &reg,
                    std::vector<pcl::PointIndices> &clusters) {

    pcl::search::Search<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(input_cloud);
    normal_estimator.setKSearch(segmentation_k_search_count);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*input_cloud, *indices);

    reg.setMinClusterSize(segmentation_min_cluster_size);
    reg.setMaxClusterSize(segmentation_max_cluster_size);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(segmentation_num_neighbors);
    reg.setInputCloud(input_cloud);
    reg.setIndices(indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(segmentation_smoothness_threshold);
    reg.setCurvatureThreshold(segmentation_curvature_threshold);

    reg.extract(clusters);
  }
};

#endif /* _POINT_CLOUD_PROCESSOR_ENGINE_HPP_ */