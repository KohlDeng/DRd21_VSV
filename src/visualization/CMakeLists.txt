cmake_minimum_required(VERSION 3.0.2)
project(visualization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  fsd_common_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(visualization src/main.cpp)
target_link_libraries(visualization ${catkin_LIBRARIES})

