add_executable(image_converter main.cpp image_converter.cpp nodehandle.cpp)
target_link_libraries(image_converter ${catkin_LIBRARIES} ${OpenMP_LIBS})
add_dependencies(image_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(image_converter PRIVATE ${OpenMP_FLAGS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(image_converter ${OpenCV_LIBRARIES})
