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
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width*cloud->height);
	for (size_t i = 0; i < cloud->points.size(); i++) {
		cloud->points[i].x = 1024.0f*rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f*rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f*rand() / (RAND_MAX + 1.0f);
	}
	PointXYZ search_point;
	search_point.x = 1024.0f*rand() / (RAND_MAX + 1.0f);
	search_point.y = 1024.0f*rand() / (RAND_MAX + 1.0f);
	search_point.z = 1024.0f*rand() / (RAND_MAX + 1.0f);
	cout << "search point:\n" << search_point.x << " " << search_point.y << " " << search_point.z << endl;

	float resolution = 128.0f;
	octree::OctreePointCloudSearch<PointXYZ> octree(resolution);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();
	vector<int> point_ids;
	vector<float> point_distance2;

	// vs - voxel neighbour search
	cout << "\nVS: resolution=" << resolution << endl;
	point_ids.clear();
	if (octree.voxelSearch(search_point, point_ids)) {
		for (size_t i = 0; i < point_ids.size(); i++) {
			cout << "points: " << cloud->points[point_ids[i]].x << " " << cloud->points[point_ids[i]].y << " " << cloud->points[point_ids[i]].z;
		}
	}

	// knn - k nearest neighbour search
	int k = 10;
	cout << "\nKNN: k=" << k << endl;
	point_ids.clear();
	point_distance2.clear();
	if (octree.nearestKSearch(search_point, k, point_ids, point_distance2) > 0) {
		for (size_t i = 0; i < point_ids.size(); i++) {
			cout << "points: " << cloud->points[point_ids[i]].x << " " << cloud->points[point_ids[i]].y << " " << cloud->points[point_ids[i]].z;
			cout << " (distance: " << sqrt(point_distance2[i]) << " )" << endl;
		}
	}

	// fdn - fixed ditance neighbour search
	float r = 256.0f*rand() / (RAND_MAX + 1.0f);
	cout << "\nFDN: r=" << r << endl;
	point_ids.clear();
	point_distance2.clear();
	if (octree.radiusSearch(search_point, r, point_ids, point_distance2)); {
		for (size_t i = 0; i < point_ids.size(); i++) {
			cout << "points: " << cloud->points[point_ids[i]].x << " " << cloud->points[point_ids[i]].y << " " << cloud->points[point_ids[i]].z;
			cout << " (distance: " << sqrt(point_distance2[i]) << " )" << endl;
		}
	}

	system("pause");
	return 0;
}
