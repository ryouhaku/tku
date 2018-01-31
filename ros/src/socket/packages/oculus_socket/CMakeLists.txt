cmake_minimum_required(VERSION 2.8.3)
project(oculus_socket)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  visualization_msgs
  #liboscpack
  ip
  osc
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall -loscpack ${CMAKE_CXX_FLAGS}")

catkin_package(
  DEPENDS ip osc
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(oculus_sender nodes/oculus_sender/oculus_sender.cpp)
#add_library(oculus_socket liboscpack)
target_link_libraries(oculus_sender ${catkin_LIBRARIES})
