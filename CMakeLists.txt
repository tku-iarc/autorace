################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(vision)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  message_generation
)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()
################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  aim.msg
  aim2.msg
  bin.msg
  color.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package( CATKIN_DEPENDS 
  std_msgs 
  message_runtime 
  std_msgs 
  roscpp 
  cv_bridge 
  image_transport
 DEPENDS
  OpenMP
)

################################################################################
# Build
################################################################################

include_directories(
  ${catkin_INCLUDE_DIRS}
)
foreach(subdir
	image_converter
)
add_subdirectory(${subdir})
endforeach()
################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
