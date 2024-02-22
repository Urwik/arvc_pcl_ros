#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <thread>

// ROS 
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>


// Type Definitions ////////////////////////////////////////////////////////////
typedef pcl::PointXYZI PointI;
typedef pcl::PointXYZL PointL;

namespace fs = std::filesystem;

const std::string CLOUDS_PATH = "/home/fran/workSpaces/arvc_ws/src/arvc_pcl_ros/data/";

  bool saved_cloud = false;



/////////////////////////////////////////////////////////////////
fs::path create_day_folder()
{
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::tm* now_tm = std::localtime(&now_c);

  // Format time into a string
  std::stringstream ss;
  ss << std::put_time(now_tm, "%Y-%m-%d");
  std::string date_str = ss.str();

  fs::path path = CLOUDS_PATH + date_str;

  if (!fs::exists(CLOUDS_PATH + date_str)) {
    fs::create_directories(CLOUDS_PATH + date_str);
  }

  return path;
}


/////////////////////////////////////////////////////////////////
fs::path getLastWrittenFile(fs::path path)
{
  std::filesystem::path latest_file;
  std::filesystem::file_time_type latest_time = std::filesystem::file_time_type::min();


  if (std::filesystem::directory_iterator(path) == std::filesystem::directory_iterator()) {
      latest_file = path / "0.pcd";
  } 
  else {

    for (const auto& entry : std::filesystem::directory_iterator(path)) {
      if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
        auto time = entry.last_write_time();
        if (time > latest_time) {
          latest_time = time;
          latest_file = entry.path();
        }
      }
    }
  }

  return latest_file;
}



/////////////////////////////////////////////////////////////////
void saveCloud(pcl::PointCloud<PointI>::Ptr cloud, fs::path path = "arvc_saved_cloud.pcd"){
  std::cout << "Saving cloud to: " << path << std::endl;

  pcl::PointCloud<PointL>::Ptr cloud_l(new pcl::PointCloud<PointL>);

  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    PointL p;
    p.x = cloud->points[i].x;
    p.y = cloud->points[i].y;
    p.z = cloud->points[i].z;
    p.label = int(cloud->points[i].intensity);
    cloud_l->push_back(p);
  }
  

  pcl::PCDWriter writer;

  writer.write<PointL>(path.string(), *cloud_l, false);

}


void saveTransform( fs::path _clouds_path, std::string _pose_id){
  tf::TransformListener listener;
  tf::StampedTransform transform;

  while (transform.frame_id_ != "gz_world" || transform.child_frame_id_ != "os1_sensor")
  {
    try
    {
      listener.lookupTransform("/gz_world","/os1_sensor", ros::Time(0), transform);
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "Transforn found" << std::endl;


  fs::path poses_file = _clouds_path.parent_path() / "poses.yaml";

  std::cout << "Saving transform to: " << poses_file << std::endl;

  YAML::Node node;

  // Load existing data if file exists
  if (std::filesystem::exists(poses_file)) {
      node = YAML::LoadFile(poses_file.string());
  }

  // Add new data
  node["pose"+_pose_id]["translation"]["x"] = transform.getOrigin().x();
  node["pose"+_pose_id]["translation"]["y"] = transform.getOrigin().y();
  node["pose"+_pose_id]["translation"]["z"] = transform.getOrigin().z();
  node["pose"+_pose_id]["rotation"]["x"] = transform.getRotation().x();
  node["pose"+_pose_id]["rotation"]["y"] = transform.getRotation().y();
  node["pose"+_pose_id]["rotation"]["z"] = transform.getRotation().z();
  node["pose"+_pose_id]["rotation"]["w"] = transform.getRotation().w();

  // Create a YAML emitter
  YAML::Emitter out;
  out << node;

  // Write the YAML data to the file
  std::ofstream fout(poses_file);
  fout << out.c_str();

}


/////////////////////////////////////////////////////////////////
void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PointCloud<PointI>::Ptr cloud_i (new pcl::PointCloud<PointI>);

  pcl::PCLPointCloud2 pcl2;
  pcl_conversions::toPCL(*input, pcl2);
  pcl::fromPCLPointCloud2(pcl2,*cloud_i);


  fs::path clouds_folder = create_day_folder();
  fs::path last_file = getLastWrittenFile(clouds_folder);
  fs::path new_cloud_path = clouds_folder / (std::to_string(std::stoi(last_file.stem()) + 1) + ".pcd");
  std::cout << "New cloud path: " << new_cloud_path << std::endl;
  
  saveCloud(cloud_i, new_cloud_path);
  saveTransform(new_cloud_path, new_cloud_path.stem());

  saved_cloud = true;
}



int main(int argc, char **argv)
{


  ros::init(argc, argv, "arvc_save_cloud");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/os1/pointCloud", 1, pcCallback);

  while(!saved_cloud){
    ros::spinOnce();
  }

  return 0;
}

