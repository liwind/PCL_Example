#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <iostream>
#include <vector>
#include <ctime>

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
	// generate random point cloud with 1000 points
	srand(time(NULL));
	PointCloud<PointXYZ>::Ptr cloud_a(new PointCloud<PointXYZ>);
	cloud_a->width = 128;
	cloud_a->height = 1;
	cloud_a->points.resize(cloud_a->width * cloud_a->height);
	for (size_t i = 0; i < cloud_a->points.size(); i++) {
		cloud_a->points[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
		cloud_a->points[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
		cloud_a->points[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
	}
	PointCloud<PointXYZ>::Ptr cloud_b(new PointCloud<PointXYZ>);
	cloud_b->width = 128;
	cloud_b->height = 1;
	cloud_b->points.resize(cloud_b->width * cloud_b->height);
	for (size_t i = 0; i < cloud_b->points.size(); i++) {
		cloud_b->points[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
		cloud_b->points[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
		cloud_b->points[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
	}

	// instance OctreePointCloudChangeDetector and add point cloud data
	float resolution = 32.0f;
	octree::OctreePointCloudChangeDetector<PointXYZ> octree(resolution);
	octree.setInputCloud(cloud_a);
	octree.addPointsFromInputCloud();
	octree.switchBuffers();
	octree.setInputCloud(cloud_b);
	octree.addPointsFromInputCloud();

	vector<int> point_ids;
	octree.getPointIndicesFromNewVoxels(point_ids);
	cout << "\nOutput different points: " << endl;
	for (size_t i = 0; i < point_ids.size(); i++) {
		cout << "points: " << cloud_b->points[point_ids[i]].x << " " << cloud_b->points[point_ids[i]].y << " " << cloud_b->points[point_ids[i]].z << endl;
	}

	system("pause");
	return 0;
}
