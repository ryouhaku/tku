cmake_minimum_required(VERSION 2.8.3)
project(oculus_socket)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  visualization_msgs
#
  velodyne_pointcloud
  jsk_recognition_msgs
  jsk_rviz_plugins
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(oculus_sender nodes/oculus_sender/oculus_sender.cpp)
target_link_libraries(oculus_sender ${catkin_LIBRARIES})
