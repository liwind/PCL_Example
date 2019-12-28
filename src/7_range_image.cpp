#include <iostream>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>

int main(int argc, char** argv)
{
	// generate random point cloud with 1000 points
	pcl::PointCloud<pcl::PointXYZ> cloud;
	for (float y = -0.5f; y <= 0.5f; y += 0.01f) {
		for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
			pcl::PointXYZ point;
			point.x = 2.0f - y;
			point.y = y;
			point.z = z;
			cloud.points.push_back(point);
		}
	}
	cloud.width = (uint32_t)cloud.points.size();
	cloud.height = 1;

	// angularResolution 1.0
	float angularResolution = (float)(1.0f * (M_PI / 180.0f));
	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel = 0.00;
	float minRange = 0.0f;
	int borderSize = 1;
	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(cloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

	std::cerr << rangeImage << "\n";

	system("pause");
	return 0;
}