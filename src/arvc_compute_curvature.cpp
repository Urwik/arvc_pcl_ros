#include <iostream>
#include <filesystem>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d_omp.h>

// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointT> PointCloud;
namespace fs = std::filesystem;

//Global Variables
PointCloud::Ptr cloud (new PointCloud);

void saveCloud(PointCloud::Ptr cloud, std::string name, bool binary){
  pcl::PCDWriter writer;
  writer.write<PointT>(name, *cloud, binary);
}

PointCloud::Ptr readCloud(fs::path path)
{
  pcl::PCDReader reader;
  PointCloud::Ptr cloud (new PointCloud);
  reader.read<PointT>(path, *cloud);

  return cloud;
}

pcl::PointCloud<PointNT>::Ptr EstimateCurvature(PointCloud::Ptr inCloud)
{
  pcl::PointCloud<PointNT>::Ptr cloudNormals (new pcl::PointCloud<PointNT>);
  
  pcl::NormalEstimationOMP<PointT, PointNT> ne;
  
  ne.setInputCloud(inCloud);
  ne.setNumberOfThreads(6);
  ne.setRadiusSearch(0.25);
  ne.compute(*cloudNormals);

  return cloudNormals;
}


int main(int argc, char **argv)
{
  fs::path path = "/home/fran/workSpaces/arvc_ws/src/arvc_pcl_ros/data/Normals.pcd";
  PointCloud::Ptr original (new PointCloud);
  original = readCloud(path);
  pcl::PointCloud<PointNT>::Ptr output (new pcl::PointCloud<PointNT>);
  output = EstimateCurvature(original);
  
  pcl::PCDWriter writer;
  writer.write<PointNT>("nomals.pcd", *output, false);

  return 0;
}

