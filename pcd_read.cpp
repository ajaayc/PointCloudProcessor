#include <iostream>
#include <pcl/common/io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace cloud_helpers {
void printCloudInfo(std::string cloud_name,
                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  std::cout << cloud_name << " has: " << cloud->width * cloud->height
            << " data points." << std::endl;
}

void printCloudPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  for (const auto &point : *cloud)
    std::cout << "    " << point.x << " " << point.y << " " << point.z
              << std::endl;
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
    addCloud(cloud_name, cloud, 0.5, 0.0, 1.0, 0.5);
  }
  void addVoxelizedCloud(std::string cloud_name,
                         pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    addCloud(cloud_name, cloud, 0.0, 0.5, 0.5, 1.0);
  }
  void addSegmentedCloud(std::string cloud_name,
                         pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    addCloud(cloud_name, cloud, 0.5, 0.5, 1.0, 1.0);
  }

  void runVisualization() {
    while (!viewer->wasStopped()) {
      viewer->spinOnce(100);
      std::this_thread::sleep_for(100ms);
    }
  }

private:
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
  std::string original_cloud_filename;
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_cloud;
  // TODO: Insert segmented cloud here

  // Outlier filtering parameters
  const int filtering_meank;
  const double filtering_stddevmulthresh;

  // Voxelization parameters
  const double voxelization_leaf_size_x;
  const double voxelization_leaf_size_y;
  const double voxelization_leaf_size_z;

public:
  PointCloudProcessorEngine(const std::string &original_cloud_filename,
                            int filtering_meank,
                            double filtering_stddevmulthresh,
                            double voxelization_leaf_size_x,
                            double voxelization_leaf_size_y,
                            double voxelization_leaf_size_z)

      : original_cloud_filename{original_cloud_filename},
        original_cloud{new pcl::PointCloud<pcl::PointXYZ>},
        filtered_cloud{new pcl::PointCloud<pcl::PointXYZ>},
        voxelized_cloud{new pcl::PointCloud<pcl::PointXYZ>},
        filtering_meank{filtering_meank},
        filtering_stddevmulthresh{filtering_stddevmulthresh},
        voxelization_leaf_size_x{voxelization_leaf_size_x},
        voxelization_leaf_size_y{voxelization_leaf_size_y},
        voxelization_leaf_size_z{voxelization_leaf_size_z} {}

  int loadOriginalCloudFromFile() {
    return loadCloudFromFile(original_cloud_filename, original_cloud);
  }

  void filterOriginalCloud() {
    filterCloud(original_cloud, filtered_cloud, filtering_meank,
                filtering_stddevmulthresh);
  }

  void voxelizeFilteredCloud() {
    voxelizeCloud(filtered_cloud, voxelized_cloud, voxelization_leaf_size_x,
                  voxelization_leaf_size_y, voxelization_leaf_size_z);
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

  std::string getOriginalCloudName() const { return "Original Cloud"; }

  std::string getFilteredCloudName() const { return "Filtered Cloud"; }

  std::string getVoxelizedCloudName() const { return "Voxelized Cloud"; }

private:
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

  void voxelizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_cloud,
                     double leaf_size_x, double leaf_size_y,
                     double leaf_size_z) {
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(input_cloud);
    voxel_grid.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    voxel_grid.filter(*voxelized_cloud);
  }

  void filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud,
                   double filtering_meank, double filtering_stddevmulthresh) {

#ifndef FILTERING_DISABLED
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(filtering_meank);
    sor.setStddevMulThresh(filtering_stddevmulthresh);
    sor.filter(*filtered_cloud);
#else
    copyPointCloud(*input_cloud, *filtered_cloud);
#endif

    return;
  }

  void segmentCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud) {
    copyPointCloud(*input_cloud, *segmented_cloud);
    return;
  }
};

int main() {
  std::string file_name = "scans.pcd";
  int filtering_meank = 30;
  double filtering_stddevmulthresh = 1.0f;
  double voxelization_leaf_size_x = 0.3f;
  double voxelization_leaf_size_y = 0.3f;
  double voxelization_leaf_size_z = 0.3f;

  PointCloudProcessorEngine eng{file_name,
                                filtering_meank,
                                filtering_stddevmulthresh,
                                voxelization_leaf_size_x,
                                voxelization_leaf_size_y,
                                voxelization_leaf_size_z};

  if (eng.loadOriginalCloudFromFile() == -1) {
    return -1;
  }

  cloud_helpers::printCloudInfo(eng.getOriginalCloudName(),
                                eng.getOriginalCloud());

  eng.filterOriginalCloud();
  cloud_helpers::printCloudInfo(eng.getFilteredCloudName(),
                                eng.getFilteredCloud());

  eng.voxelizeFilteredCloud();
  cloud_helpers::printCloudInfo(eng.getVoxelizedCloudName(),
                                eng.getVoxelizedCloud());

  std::string original_cloud_name = eng.getOriginalCloudName();
  std::string filtered_cloud_name = eng.getFilteredCloudName();
  std::string voxelized_cloud_name = eng.getVoxelizedCloudName();

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr original_cloud =
      eng.getOriginalCloud();
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr filtered_cloud =
      eng.getFilteredCloud();
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr voxelized_cloud =
      eng.getVoxelizedCloud();

  // pcl::PointCloud<pcl::PointXYZ>::ConstPtr segmented_cloud =
  //     eng.getSegmentedCloud();

  PointCloudVisualizationManager visualization_manager;

  visualization_manager.addOriginalCloud(original_cloud_name, original_cloud);
  visualization_manager.addFilteredCloud(filtered_cloud_name, filtered_cloud);
  visualization_manager.addVoxelizedCloud(voxelized_cloud_name,
                                          voxelized_cloud);

  visualization_manager.runVisualization();

  return (0);
}
