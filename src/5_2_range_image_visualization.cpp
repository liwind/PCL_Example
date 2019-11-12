#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

using namespace std;
using namespace pcl;

int user_data = 0;
void viewerOneOff(visualization::PCLVisualizer& viewer);
void viewerPsycho(visualization::PCLVisualizer& viewer);

void setViewerPose(visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
	viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2], look_at_vector[0], look_at_vector[1], look_at_vector[2], up_vector[0], up_vector[1], up_vector[2]);
	//viewer.camera.pos[0] = pos_vector[0];
	//viewer.camera_.pos[1] = pos_vector[1];
	//viewer.camera_.pos[2] = pos_vector[2];
	//viewer.camera_.focal[0] = look_at_vector[0];
	//viewer.camera_.focal[1] = look_at_vector[1];
	//viewer.camera_.focal[2] = look_at_vector[2];
	//viewer.camera_.view[0] = up_vector[0];
	//viewer.camera_.view[1] = up_vector[1];
	//viewer.camera_.view[2] = up_vector[2];
	viewer.updateCamera();
}

int main(int argc, char** argv)
{
	bool live_update = false;

	// read point cloud data and instance CloudViewer
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	io::loadPCDFile("../data/room_scan1.pcd", *cloud);
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
	RangeImage::CoordinateFrame coordinate_frame = RangeImage::CAMERA_FRAME;
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	RangeImage::Ptr range_image(new RangeImage);
	range_image->createFromPointCloud(*cloud, 0.5f, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

	visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	visualization::PointCloudColorHandlerCustom<PointWithRange> range_image_color_handler(range_image, 0, 0, 0);
	viewer.addPointCloud(range_image, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");

	viewer.initCameraParameters();
	setViewerPose(viewer, range_image->getTransformationToWorldSystem());
	visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(*range_image);

	while (!viewer.wasStopped())
	{
		range_image_widget.spinOnce();
		viewer.spinOnce();
		pcl_sleep(0.01);
		if (live_update)
		{
			scene_sensor_pose = viewer.getViewerPose();
			range_image->createFromPointCloud(*cloud, 0.5f, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
			range_image_widget.showRangeImage(*range_image);
		}
	}

	system("pause");
	return 0;
}
