cmake_minimum_required(VERSION 2.8.3)
project(osc)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES osc
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
)

add_library(osc 
  src/OscOutboundPacketStream.cpp
  src/OscPrintReceivedElements.cpp
  src/OscReceivedElements.cpp
  src/OscTypes.cpp
)

#############
## Install ##
#############

#install(
#  DIRECTORY include
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.hh"
#)
