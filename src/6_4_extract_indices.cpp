#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
int
main(int argc, char** argv)
{
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2()), cloud_filtered_blob(new pcl::PCLPointCloud2());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	// read pcd data
	pcl::PCDReader reader;
	reader.read("../data/table_scene_lms400.pcd", *cloud_blob);
	std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
	// create filter for downsample
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filtered_blob);
	// convert to template xyz point cloud data
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);
	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
	// write downsampled data
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("../data/table_scene_lms400_downsampled.pcd", *cloud_filtered, false);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	// create segment instance
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.01);
	// 创建滤波器对象
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	int i = 0, nr_points = (int)cloud_filtered->points.size();
	// 当还有30%原始点云数据时
	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		// 从余下的点云中分割最大平面组成部分
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}
		// 分离内层
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
		std::stringstream ss;
		ss << "../data/table_scene_lms400_plane_" << i << ".pcd";
		writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);
		// 创建滤波器对象
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);
		i++;
	}

	system("pause");
	return (0);
}
