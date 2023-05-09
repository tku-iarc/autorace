cmake_minimum_required(VERSION 3.0.2)
project(lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES lib
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

# node: node_test
set(${PROJECT_NAME}_NT_LIST
  ${CMAKE_CURRENT_LIST_DIR}/src/node_handle.cpp
  ${CMAKE_CURRENT_LIST_DIR}/test/node_test.cpp
)
add_executable(node_test ${${PROJECT_NAME}_NT_LIST})
target_link_libraries(node_test ${catkin_LIBRARIES})

# add_dependencies(node_test ${catkin_EXPORTED_TARGETS})
