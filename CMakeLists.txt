cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(object_matching)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (object_matching src/object_matching.cpp)
target_link_libraries (object_matching ${PCL_LIBRARIES})

#add_executable (pcd_read pcd_read.cpp)
#add_executable (file_conv pcd_exm.cpp)
#target_link_libraries (pcd_read ${PCL_LIBRARIES})
#target_link_libraries (file_conv ${PCL_LIBRARIES})
