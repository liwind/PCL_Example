cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(pcl_example)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(1_hello_pcl 1_hello_pcl.cpp)
target_link_libraries(1_hello_pcl ${PCL_LIBRARIES})

add_executable(2_pclio_pcd 2_pclio_pcd.cpp)
target_link_libraries(2_pclio_pcd ${PCL_LIBRARIES})

add_executable(3_kdtree_search 3_kdtree_search.cpp)
target_link_libraries(3_kdtree_search ${PCL_LIBRARIES})

add_executable(4_1_octree_search 4_1_octree_search.cpp)
target_link_libraries(4_1_octree_search ${PCL_LIBRARIES})

add_executable(5_1_cloud_viewer 5_1_cloud_viewer.cpp)
target_link_libraries(5_1_cloud_viewer ${PCL_LIBRARIES})

add_executable(5_2_range_image_visualization 5_2_range_image_visualization.cpp)
target_link_libraries(5_2_range_image_visualization ${PCL_LIBRARIES})

add_executable(5_3_pcl_visualizer_demo  5_3_pcl_visualizer_demo.cpp)
target_link_libraries(5_3_pcl_visualizer_demo  ${PCL_LIBRARIES})