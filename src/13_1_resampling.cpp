#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// load a test pcd file
	pcl::io::loadPCDFile<pcl::PointXYZ>("../data/bun0.pcd", *cloud);
	// create kdtree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	// create a pointnormal for mls output
	pcl::PointCloud<pcl::PointNormal> mls_points;
	// create object of mls
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);
	// prosess
	mls.process(mls_points);
	pcl::io::savePCDFile("../data/bun0-mls.pcd", mls_points);

	system("pause");
	return(0);
}
