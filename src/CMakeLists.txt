cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(pcl_example)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(0_my_test 0_my_test.cpp)
target_link_libraries(0_my_test ${PCL_LIBRARIES})

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

add_executable(8_key_point.cpp  8_key_point.cpp)
target_link_libraries(8_key_point.cpp  ${PCL_LIBRARIES})

add_executable(9_1_normal_estimation  9_1_normal_estimation.cpp)
target_link_libraries(9_1_normal_estimation  ${PCL_LIBRARIES})

add_executable(9_2_normal_estimation_integral_image  9_2_normal_estimation_integral_image.cpp)
target_link_libraries(9_2_normal_estimation_integral_image  ${PCL_LIBRARIES})

add_executable(10_1_iterative_closest_point  10_1_iterative_closest_point.cpp)
target_link_libraries(10_1_iterative_closest_point  ${PCL_LIBRARIES})

add_executable(10_2_pairwise_incremental_registration  10_2_pairwise_incremental_registration.cpp)
target_link_libraries(10_2_pairwise_incremental_registration  ${PCL_LIBRARIES})

add_executable(10_3_normal_distributions_transform  10_3_normal_distributions_transform.cpp)
target_link_libraries(10_3_normal_distributions_transform  ${PCL_LIBRARIES})

add_executable(11_random_sample_consensus  11_random_sample_consensus.cpp)
target_link_libraries(11_random_sample_consensus  ${PCL_LIBRARIES})

add_executable(12_1_planar_segmentation  12_1_planar_segmentation.cpp)
target_link_libraries(12_1_planar_segmentation  ${PCL_LIBRARIES})

add_executable(12_2_cylinder_segmentation  12_2_cylinder_segmentation.cpp)
target_link_libraries(12_2_cylinder_segmentation  ${PCL_LIBRARIES})

add_executable(12_3_cluster_extraction  12_3_cluster_extraction.cpp)
target_link_libraries(12_3_cluster_extraction  ${PCL_LIBRARIES})

add_executable(13_1_resampling  13_1_resampling.cpp)
target_link_libraries(13_1_resampling  ${PCL_LIBRARIES})

add_executable(13_2_concave_hull_2d  13_2_concave_hull_2d.cpp)
target_link_libraries(13_2_concave_hull_2d  ${PCL_LIBRARIES})

add_executable(13_3_greedy_projection  13_3_greedy_projection.cpp)
target_link_libraries(13_3_greedy_projection  ${PCL_LIBRARIES})

add_executable(14_correspondence_grouping  14_correspondence_grouping.cpp)
target_link_libraries(14_correspondence_grouping  ${PCL_LIBRARIES})