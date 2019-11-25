#include <iostream>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
	srand(time(0));
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);
	//generate random point cloud data
	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = rand() / (RAND_MAX + 1.0f) - 0.5;
		cloud->points[i].y = rand() / (RAND_MAX + 1.0f) - 0.5;
		cloud->points[i].z = rand() / (RAND_MAX + 1.0f) - 0.5;
	}
	cerr << "Cloud before filtering: " << endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << endl;
	
	// create filter instance
	PassThrough<PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	//pass.setFilterLimitsNegative (true);
	pass.filter(*cloud_filtered);

	cerr << "Cloud after filtering: " << endl;
	for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
		cerr << "    " << cloud_filtered->points[i].x << " "
		<< cloud_filtered->points[i].y << " "
		<< cloud_filtered->points[i].z << endl;

	system("pause");
	return (0);
}
