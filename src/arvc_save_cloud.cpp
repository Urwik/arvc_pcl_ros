#include <iostream>

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


// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;


//Global Variables
PointCloud::Ptr cloud (new PointCloud);

bool saved_cloud = false;

void saveCloud(PointCloud::Ptr cloud){
  pcl::PCDWriter writer;
  writer.write<PointT>("arvc_saved_cloud.pcd", *cloud, false);
  saved_cloud = true;
}


void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);

  PointCloud::Ptr temp_cloud (new PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  *cloud = *temp_cloud;

  saveCloud(cloud);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "arvc_save_cloud");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/os1/pointCloud", 1, pcCallback);

  while(!saved_cloud)
    ros::spinOnce();

  return 0;
}

