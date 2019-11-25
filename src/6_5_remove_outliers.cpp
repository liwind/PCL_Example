#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	// read pcd point cloud 
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>("../data/table_scene_lms400.pcd", *cloud);
	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;
	pcl::PCDWriter writer;

	// filter
	std::cerr << "statitical removal: sd=1.0" << std::endl;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	cloud_filtered->clear();
	sor.filter(*cloud_filtered);
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	writer.write<pcl::PointXYZ>("../data/table_scene_lms400_s_inliers.pcd", *cloud_filtered, false);
	sor.setNegative(true);
	cloud_filtered->clear();
	sor.filter(*cloud_filtered);
	writer.write<pcl::PointXYZ>("../data/table_scene_lms400_s_outliers.pcd", *cloud_filtered, false);

	std::cerr << "radius removal: r=0.005" << std::endl;
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
	ror.setInputCloud(cloud);
	ror.setRadiusSearch(0.005);
	ror.setMinNeighborsInRadius(2);
	cloud_filtered->clear();
	ror.filter(*cloud_filtered);
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	writer.write<pcl::PointXYZ>("../data/table_scene_lms400_r_inliers.pcd", *cloud_filtered, false);
	ror.setNegative(true);
	cloud_filtered->clear();
	ror.filter(*cloud_filtered);
	writer.write<pcl::PointXYZ>("../data/table_scene_lms400_r_outliers.pcd", *cloud_filtered, false);

	std::cerr << "conditional removal: y>0.0" << std::endl;
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
		new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -0.05)));
	pcl::ConditionalRemoval<pcl::PointXYZ> cdr;
	cdr.setCondition(range_cond);
	cdr.setInputCloud(cloud);
	cdr.setKeepOrganized(true);
	cloud_filtered->clear();
	cdr.filter(*cloud_filtered);
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	writer.write<pcl::PointXYZ>("../data/table_scene_lms400_c_inliers.pcd", *cloud_filtered, false);
	
	system("pause");
	return (0);
}
