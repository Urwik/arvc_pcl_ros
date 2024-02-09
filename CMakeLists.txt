cmake_minimum_required(VERSION 3.0.2)
project(arvc_pcl_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  # sensor_msgs
  # std_msgs
  # pcl_conversions
  # pcl_ros
  arvc_utils
)

find_package(PCL 1.10 EXACT REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES arvc_pcl_ros
 CATKIN_DEPENDS roscpp sensor_msgs std_msgs arvc_utils
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})
add_definitions(${catkin_DEFINITIONS})


# add_executable(arvc_save_cloud src/arvc_save_cloud.cpp)
# target_link_libraries(arvc_save_cloud ${PCL_LIBRARIES} ${catkin_LIBRARIES})

# add_executable(arvc_plot_cloud_by_field src/arvc_plot_cloud_by_field.cpp)
# target_link_libraries(arvc_plot_cloud_by_field ${PCL_LIBRARIES} ${catkin_LIBRARIES})

# add_executable(arvc_plot_by_index src/arvc_plot_by_index.cpp)
# target_link_libraries(arvc_plot_by_index ${PCL_LIBRARIES} ${catkin_LIBRARIES})

# add_executable(arvc_compute_curvature src/arvc_compute_curvature.cpp)
# target_link_libraries(arvc_compute_curvature ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(arvc_pcl_ros_node src/main.cpp)
target_link_libraries(arvc_pcl_ros_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})