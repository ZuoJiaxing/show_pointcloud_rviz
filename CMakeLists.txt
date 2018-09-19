cmake_minimum_required(VERSION 2.8.3)
project(show_pointcloud)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(PCL REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS}  ${catkin_INCLUDE_DIRS})

# Describe catkin Project
catkin_package(
    DEPENDS Boost PCL
    CATKIN_DEPENDS   pcl_ros   roscpp   rospy   sensor_msgs   std_msgs
    INCLUDE_DIRS src
)


link_directories(${PCL_LIBRARY_DIRS})
add_executable(readin src/pcl_read.cpp)
target_link_libraries(readin ${catkin_LIBRARIES} ${PCL_LIBRARIES})
