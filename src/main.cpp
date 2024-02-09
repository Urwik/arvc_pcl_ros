#include <iostream>
#include <arvc_utils/common.hpp>
#include "utils_legacy.hpp"

int main(int argc, char const *argv[])
{
    arvc::Console console;
    console.debug("Hello, World!", "GREEN");

    arvc::Color color;

    arvc::Plane plane;

    arvc::Axes3D axes;

    arvc::Viewer viewer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud = arvc::readPointCloud<pcl::PointXYZ>("/home/fran/workSpaces/arvc_ws/src/arvc_pcl_ros/data/arvc_saved_cloud.pcd");

    viewer.setBackgroundColor(0,0,0);
    viewer.addCloud(cloud, GREEN_COLOR);
    viewer.show();


    return 0;
}
