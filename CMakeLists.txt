cmake_minimum_required(VERSION 2.8.3)
project(optitrack_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES optitrack_bridge
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(SampleClient src/SampleClient.cpp)
target_link_libraries(SampleClient
  ${PROJECT_SOURCE_DIR}/lib/libNatNetLibShared.so
  ${catkin_LIBRARIES}
)

add_executable(optitrack_bridge_node
  src/optitrack_bridge_node.cpp
  src/NatNetWrapper.cpp
  src/LinearKalmanFilter.cpp
)
target_link_libraries(optitrack_bridge_node
  ${PROJECT_SOURCE_DIR}/lib/libNatNetLibShared.so
  ${catkin_LIBRARIES}
)