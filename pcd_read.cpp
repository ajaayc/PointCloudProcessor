#include <iostream>
#include <pcl/common/io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <thread>

using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr
createSimpleVisualizer(std::string cloud_name,
                       pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer(cloud_name + " Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_name);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->setSize(1200, 800);
  return (viewer);
}

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

void displayCloudWithVisualizer(
    pcl::visualization::PCLVisualizer::Ptr visualizer) {
  while (!visualizer->wasStopped()) {
    visualizer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  }
}

void createVisualizerAndDisplayCloud(
    const std::string &cloud_name,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  pcl::visualization::PCLVisualizer::Ptr visualizer =
      createSimpleVisualizer(cloud_name, cloud);
  displayCloudWithVisualizer(visualizer);
}

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
    //TODO: To be implemented
    copyPointCloud(*input_cloud, *filtered_cloud);
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

  printCloudInfo(eng.getOriginalCloudName(), eng.getOriginalCloud());

  eng.filterOriginalCloud();
  printCloudInfo(eng.getFilteredCloudName(), eng.getFilteredCloud());

  eng.voxelizeFilteredCloud();
  printCloudInfo(eng.getVoxelizedCloudName(), eng.getVoxelizedCloud());

  std::string original_cloud_name = eng.getOriginalCloudName();
  std::string filtered_cloud_name = eng.getFilteredCloudName();
  std::string voxelized_cloud_name = eng.getVoxelizedCloudName();

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr original_cloud =
      eng.getOriginalCloud();
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr filtered_cloud =
      eng.getFilteredCloud();
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr voxelized_cloud =
      eng.getVoxelizedCloud();

  createVisualizerAndDisplayCloud(original_cloud_name, original_cloud);

  return (0);
}
