#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

int main(int argc, char** argv)
{
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

	pcl::PCDReader reader;
	reader.read("../data/table_scene_lms400.pcd", *cloud); 
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").\n";

	pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.1f, 0.1f, 0.1f);
	vg.filter(*cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";
	pcl::PCDWriter writer;
	writer.write("../data/table_scene_lms400_downsampled.pcd", *cloud_filtered,
		Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
	
	system("pause");
	return (0);
}
