#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
	// read point cloud data from pcd file
	PointCloud<PointXYZ> cloud_a; // create and instance a shared ptr for points
	if (io::loadPCDFile<PointXYZ>("../data/test_pcd.pcd", cloud_a) == -1) {
		PCL_ERROR("could not open pcl file\n");
		return (-1);
	}
	cout << " read cloud_a  " << cloud_a.points.size() << " points: " << endl;
	for (size_t i = 0; i < cloud_a.points.size(); ++i) {
		cout << " " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << endl;
	}

	// write point cloud data to pcd file
	PointCloud<PointXYZ> cloud_b;
	cloud_b.width = 3;
	cloud_b.height = 1;
	cloud_b.is_dense = false;
	cloud_b.points.resize(cloud_b.width*cloud_b.height);
	for (size_t i = 0; i < cloud_b.points.size(); i++) {
		cloud_b.points[i].x = 1024.0f*rand() / (RAND_MAX + 1.0f);
		cloud_b.points[i].y = 1024.0f*rand() / (RAND_MAX + 1.0f);
		cloud_b.points[i].z = 1024.0f*rand() / (RAND_MAX + 1.0f);
	}
	io::savePCDFileASCII("../data/my_pcd.pcd", cloud_b);
	cout << "\n write cloud_b  " << cloud_b.points.size() << " points: " << endl;
	for (size_t i = 0; i < cloud_b.points.size(); ++i) {
		cout << " " << cloud_b.points[i].x << " " << cloud_b.points[i].y << " " << cloud_b.points[i].z << endl;
	}

	// join point cloud
	// simplily join two point cloud
	PointCloud<PointXYZ> cloud_c;
	cloud_c = cloud_a;
	cloud_c += cloud_b;
	cout << "\n join two point cloud - cloud_c = cloud_a + cloud_b  " << cloud_c.points.size() << " points: " << endl;
	for (size_t i = 0; i < cloud_c.points.size(); ++i) {
		cout << " " << cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << endl;
	}
	// concatenate field to a point cloud with same number of points
	PointCloud<Normal> n_cloud_b;
	n_cloud_b.width = cloud_a.width;
	n_cloud_b.height = cloud_a.height;
	n_cloud_b.points.resize(n_cloud_b.width*n_cloud_b.height);
	for (size_t i = 0; i < n_cloud_b.points.size(); i++) {
		n_cloud_b.points[i].normal_x = 1024.0f*rand() / (RAND_MAX + 1.0f);
		n_cloud_b.points[i].normal_y = 1024.0f*rand() / (RAND_MAX + 1.0f);
		n_cloud_b.points[i].normal_z = 1024.0f*rand() / (RAND_MAX + 1.0f);
	}
	PointCloud<PointNormal> pn_cloud_c;
	concatenateFields(cloud_a, n_cloud_b, pn_cloud_c);
	cout << "\n concatenate normal - pn_cloud_c = cloud_a + n_cloud_b  " << pn_cloud_c.points.size() << " points: " << endl;
	for (size_t i = 0; i < pn_cloud_c.points.size(); ++i) {
		cout << " " << pn_cloud_c.points[i].x << " " << pn_cloud_c.points[i].y << " " << pn_cloud_c.points[i].z
			<< pn_cloud_c.points[i].normal_x << " " << pn_cloud_c.points[i].normal_y << " " << pn_cloud_c.points[i].normal_z << endl;
	}

	system("pause");
	return 0;
}