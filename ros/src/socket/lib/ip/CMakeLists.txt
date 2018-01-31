cmake_minimum_required(VERSION 2.8.3)
project(ip)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ip
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
)

add_library(ip
  src/IpEndpointName.cpp
  src/NetworkingUtils.cpp
  src/UdpSocket.cpp
)

#############
## Install ##
#############

#install(
#  DIRECTORY include
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.hh"
#)
