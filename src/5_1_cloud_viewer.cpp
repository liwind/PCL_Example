#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;

int user_data = 0;
void viewerOneOff(visualization::PCLVisualizer& viewer);
void viewerPsycho(visualization::PCLVisualizer& viewer);

int main(int argc, char** argv)
{
	// read point cloud data and instance CloudViewer
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	io::loadPCDFile("../data/maize.pcd", *cloud);
	visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud);

	// regester callback run once
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	// regester callback run per renderer
	//viewer.runOnVisualizationThread(viewerPsycho);

	while (!viewer.wasStopped()) {
		user_data++;
	}

	system("pause");
	return 0;
}

void viewerOneOff(visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	PointXYZ o(1.0, 0, 0);
	viewer.addSphere(o, 0.25, "sphere", 0);
	cout << "i only run once" << endl;
}

void viewerPsycho(visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("sphere", 0);
	viewer.addText(ss.str(), 200, 300, "", 0);
	//FIXME: possible race condition here:
	user_data++;
}