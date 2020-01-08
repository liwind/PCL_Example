// This is test code for tools of PCL
// used to understand and apply PCL Library
//
#include <iostream>
#include <pcl/io/ascii_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	
	// read pcd point cloud 
	pcl::io::loadPCDFile<pcl::PointXYZ>("../data/samp31.pcd", *cloud);

	std::cerr << "\nCloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;
	pcl::IndicesConstPtr ids;

	//pcl::visualization::CloudViewer viewer("Cloud Viewer");
	//viewer.showCloud(cloud);

	// filter
	std::cerr << "\nstatitical removal: mk=8 sd=1.0" << std::endl;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor(true);
	sor.setInputCloud(cloud);
	sor.setMeanK(8);
	sor.setStddevMulThresh(1.0);
	cloud_filtered->clear();
	sor.filter(*cloud_filtered);
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	ids = sor.getRemovedIndices();
	int id = 0;
	std::cout << "\ninput id: ";
	std::cin >> id;
	std::cerr << "\nout ids size: " << ids->at(id) << std::endl;

	system("pause");
	return EXIT_SUCCESS;
}
