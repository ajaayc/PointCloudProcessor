cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcl_reader_and_visualizer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(../common/include)

add_executable (pcd_read src/pcd_read.cpp)
target_link_libraries (pcd_read ${PCL_LIBRARIES})
