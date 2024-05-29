#include <fstream>
#include <iostream>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <string>
#include <thread>

using namespace std::chrono_literals;

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

class PointCloudVisualizationManager {
  pcl::visualization::PCLVisualizer::Ptr viewer;

  size_t num_clouds;

public:
  PointCloudVisualizationManager()
      : viewer{new pcl::visualization::PCLVisualizer("3D Viewer")}, num_clouds{
                                                                        0} {
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setSize(1200, 800);
  }

  void addOriginalCloud(std::string cloud_name,
                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    addCloud(cloud_name, cloud, 0.0, 0.0, 0.5, 0.5);
  }
  void addFilteredCloud(std::string cloud_name,
                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
#ifndef OBTAIN_FILTERED_RESULT_FROM_FILE
    addCloud(cloud_name, cloud, 0.5, 0.0, 1.0, 0.5);
#else
    addCloud(cloud_name + " (Pre-computed filter result from file)", cloud, 0.5,
             0.0, 1.0, 0.5);
#endif
  }
  void addVoxelizedCloud(std::string cloud_name,
                         pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
#ifndef VOXELIZATION_DISABLED
    addCloud(cloud_name, cloud, 0.0, 0.5, 0.5, 1.0);
#else
    addCloud(cloud_name + " (No voxelization performed)", cloud, 0.0, 0.5, 0.5,
             1.0);
#endif
  }

  void addSegmentedCloud(std::string cloud_name, size_t num_clusters,
                         pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    addColoredSegmentedCloud(cloud_name, num_clusters, cloud, 0.5, 0.5, 1.0,
                             1.0);
  }

  void runVisualization() {
    while (!viewer->wasStopped()) {
      viewer->spinOnce(100);
      std::this_thread::sleep_for(100ms);
    }
  }

private:
  void
  addColoredSegmentedCloud(std::string cloud_name, unsigned num_clusters,
                           pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                           double x_min, double y_min, double x_max,
                           double y_max) {
    int view_port_id = num_clouds++;

    viewer->createViewPort(x_min, y_min, x_max, y_max, view_port_id);
    viewer->setBackgroundColor(0, 0, 0, view_port_id);
    std::string graph_text =
        cloud_name + " (" + std::to_string(num_clusters) + " clusters)";
    viewer->addText(graph_text, 10, 10, view_port_id + " text", view_port_id);
    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
    //     cloud);
    //     viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, cloud_name,
    //                                             view_port_id);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, cloud_name, view_port_id);

    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloud_name);
  }

  void addCloud(std::string cloud_name,
                pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, double x_min,
                double y_min, double x_max, double y_max) {
    int view_port_id = num_clouds++;

    viewer->createViewPort(x_min, y_min, x_max, y_max, view_port_id);
    viewer->setBackgroundColor(0, 0, 0, view_port_id);
    viewer->addText(cloud_name, 10, 10, view_port_id + " text", view_port_id);
    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
    //     cloud);
    //     viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, cloud_name,
    //                                             view_port_id);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_name, view_port_id);
  }
};

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

  //NOTE: Ptr is copied, so if changes to original pointer are made
  //outside of the class, the changes will propagate to this object.
  void setOriginalCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr new_original_cloud)
  {
     original_cloud = new_original_cloud;
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
    cloud_helpers::writeCloudToFile(filtered_cloud_filename, filtered_cloud);
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

int main() {
  std::string original_cloud_file_name = "scans.pcd";
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
