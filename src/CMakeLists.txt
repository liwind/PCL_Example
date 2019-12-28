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

add_executable(4_2_octree_change_detection 4_2_octree_change_detection.cpp)
target_link_libraries(4_2_octree_change_detection ${PCL_LIBRARIES})

add_executable(5_1_cloud_viewer 5_1_cloud_viewer.cpp)
target_link_libraries(5_1_cloud_viewer ${PCL_LIBRARIES})

add_executable(5_2_range_image_visualization 5_2_range_image_visualization.cpp)
target_link_libraries(5_2_range_image_visualization ${PCL_LIBRARIES})

add_executable(5_3_pcl_visualizer_demo  5_3_pcl_visualizer_demo.cpp)
target_link_libraries(5_3_pcl_visualizer_demo  ${PCL_LIBRARIES})

add_executable(6_1_pass_through  6_1_pass_through.cpp)
target_link_libraries(6_1_pass_through  ${PCL_LIBRARIES})

add_executable(6_2_voxel_grid  6_2_voxel_grid.cpp)
target_link_libraries(6_2_voxel_grid  ${PCL_LIBRARIES})

add_executable(6_3_project_inliers  6_3_project_inliers.cpp)
target_link_libraries(6_3_project_inliers  ${PCL_LIBRARIES})

add_executable(6_4_extract_indices  6_4_extract_indices.cpp)
target_link_libraries(6_4_extract_indices  ${PCL_LIBRARIES})

add_executable(6_5_remove_outliers  6_5_remove_outliers.cpp)
target_link_libraries(6_5_remove_outliers  ${PCL_LIBRARIES})

add_executable(7_range_image  7_range_image.cpp)
target_link_libraries(7_range_image  ${PCL_LIBRARIES})

add_executable(8_key_point  8_key_point.cpp)
target_link_libraries(8_key_point  ${PCL_LIBRARIES})
