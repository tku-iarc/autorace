cmake_minimum_required(VERSION 3.0.2)
project(lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/node_handle.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# node: node_test
set(${PROJECT_NAME}_NT_LIST
  src/node_handle.cpp
  test/node_test.cpp
)
add_executable(node_test ${${PROJECT_NAME}_NT_LIST})
target_link_libraries(node_test ${catkin_LIBRARIES})

# add_dependencies(node_test ${catkin_EXPORTED_TARGETS})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
