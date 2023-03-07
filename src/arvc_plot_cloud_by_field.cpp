#include <iostream>
#include <filesystem>

// ROS 
#include <ros/ros.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>

// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::visualization::PCLVisualizer pclVis;
namespace fs = std::filesystem;



int main(int argc, char **argv)
{
  fs::path root_dir, file_path;
  std::string filename;
  root_dir = "/home/fran/workSpaces/arvc_ws/src/arvc_pcl_ros/data";
  filename = "test_pcd.pcd";
  file_path = root_dir / filename;

  PointCloud::Ptr cloud (new PointCloud);
  pcl::PCDReader reader;
  reader.read(file_path, *cloud);

  pclVis::Ptr visualizer (new pclVis);
  pcl::visualization::PointCloudColorHandlerGenericField<PointT> handler(cloud, "intensity");
  visualizer->addPointCloud(cloud, handler, "cloud");

  while (!visualizer->wasStopped())
  {
    visualizer->spinOnce(100);
  }

  return 0;  
}

