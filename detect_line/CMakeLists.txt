cmake_minimum_required(VERSION 3.0.2)
project(detect_line)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  lib
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES detect_line
 CATKIN_DEPENDS roscpp std_msgs cv_bridge
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# node: node_test
set(${PROJECT_NAME}_NT_LIST
  src/detect_line.cpp
  test/node_test.cpp
)
add_executable(node_test ${${PROJECT_NAME}_NT_LIST})
target_link_libraries(node_test ${catkin_LIBRARIES} ${lib_LIBRARIES})
