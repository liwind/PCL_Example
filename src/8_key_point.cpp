#include <iostream>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/console/parse.h>

void setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2], up_vector[0], up_vector[1], up_vector[2]);
	viewer.updateCamera();
}

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
	pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;

	range_image.createFromPointCloud(cloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

	// 创建3D点云可视化窗口，并显示点云
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange>range_image_color_handler(range_image_ptr, 0, 0, 0);
	viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	//viewer.addCoordinateSystem (1.0f);
	//PointCloudColorHandlerCustom<PointType>point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
	//viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
	viewer.initCameraParameters();
	setViewerPose(viewer, range_image.getTransformationToWorldSystem());
	// 显示距离图像
	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(range_image);

	//提取NARF关键点
	float support_size = 0.2f;
	pcl::RangeImageBorderExtractor range_image_border_extractor;
	pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
	narf_keypoint_detector.setRangeImage(&range_image);
	narf_keypoint_detector.getParameters().support_size = support_size;
	narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
	narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

	pcl::PointCloud<int>keypoint_indices;
	narf_keypoint_detector.compute(keypoint_indices);
	std::cout << "Found " << keypoint_indices.points.size() << " key points.\n";
	//在距离图像显示组件内显示关键点
	//for (size_ti=0; i<keypoint_indices.points.size (); ++i)
	//range_image_widget.markPoint (keypoint_indices.points[i]%range_image.width,
	//keypoint_indices.points[i]/range_image.width);
	//在3D窗口中显示关键点
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
	keypoints.points.resize(keypoint_indices.points.size());
	for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
		keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>keypoints_color_handler(keypoints_ptr, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
	// 主循环
	while (!viewer.wasStopped())
	{
		range_image_widget.spinOnce();// process GUI events
		viewer.spinOnce();
		pcl_sleep(0.01);
	}
	

	system("pause");
	return 0;
}