#include <iostream>
#include <filesystem>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::visualization::PCLVisualizer pclVis;

namespace fs = std::filesystem;

pclVis::Ptr visualizer (new pclVis("Visualizer"));


struct myRGB{
  int r;
  int g;
  int b;
};


pcl::PointIndices::Ptr findValue(PointCloud::Ptr cloud, const int value)
{
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  int index=0;
  for (auto point : *cloud)
  {
    if(point.intensity == value)
    {
      inliers->indices.push_back(index);
    }
    index++;
  }
  return inliers;
}


myRGB randRGB()
{
  myRGB color;
  color.r = rand() % 256;
  color.g = rand() % 256;
  color.b = rand() % 256;

  return color;
}


void plotCloud(fs::path path_to_cloud, int min_points = 20, int max_points = 1000)
{
  PointCloud::Ptr cloud (new PointCloud);
  PointCloud::Ptr tmp_cloud (new PointCloud);

  pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  
  pcl::PCDReader reader;
  reader.read(path_to_cloud, *cloud);

  std::stringstream ss;
  
  for (size_t i = 0; i < 6*40; i++)
  {
    ss.str("");
    ss << "cloud_" << i;
    indices.reset(new pcl::PointIndices);
    indices = findValue(cloud, i);
    // if (indices->indices.size() > min_points && indices->indices.size() < max_points )
    if (indices->indices.size() > min_points)
    {
      extract.setIndices(indices);
      extract.filter(*tmp_cloud);
      myRGB color = randRGB();
      pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (tmp_cloud, color.r, color.g, color.b);
      visualizer->addPointCloud<PointT>(tmp_cloud, single_color, ss.str().c_str());
    }
  
  }
}

int main(int argc, char **argv)
{
  fs::path abs_path, current_path;
  std::string filename;

  std::string input = argv[1];
  abs_path = fs::path(input);

  plotCloud(abs_path, 10, 10000000);

  while (!visualizer->wasStopped())
  {
    visualizer->spinOnce(100, true);
  }

  return 0;  
}


