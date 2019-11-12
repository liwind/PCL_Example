#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_projected(new PointCloud<PointXYZ>);

	// Fill in the cloud data
	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	cerr << "Hello PCL!\nCloud before projection: " << endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		cerr << " " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;

	// Create a set of planar coefficients with X=Y=0,Z=1
	ModelCoefficients::Ptr coefficients(new ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	// Create the filtering object
	ProjectInliers<PointXYZ> proj;
	proj.setModelType(SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);

	cerr << "\nCloud after projection: " << endl;
	for (size_t i = 0; i < cloud_projected->points.size(); ++i)
		cerr << " " << cloud_projected->points[i].x << " "
		<< cloud_projected->points[i].y << " "
		<< cloud_projected->points[i].z << endl;
	
	system("pause");
	return (0);
}
