#ifndef _POINT_CLOUD_VISUALIZATION_MANAGER_HPP_
#define _POINT_CLOUD_VISUALIZATION_MANAGER_HPP_

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

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

#endif /* _POINT_CLOUD_VISUALIZATION_MANAGER_HPP_ */
