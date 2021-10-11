cmake_minimum_required(VERSION 3.0.2)
project(perception_slam_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  fsd_msgs
  roscpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS pcl_conversions pcl_msgs pcl_ros
  DEPENDS
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

include_directories( /usr/include/eigen3 )

add_executable(${PROJECT_NAME}
  src/psi_node.cpp
  src/psi.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

