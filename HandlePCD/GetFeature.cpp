#define myPointXYZ pcl::PointCloud<pcl::PointXYZ>
#include "GetFeature.h"
#include <iostream>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h> 

using namespace pcl;
using std::cout;
using std::endl;

myPointXYZ::Ptr downSampleVoxelization(myPointXYZ::Ptr source_cloud) {
	myPointXYZ::Ptr result_cloud(new myPointXYZ);
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(source_cloud);
	filter.setLeafSize(5,5,5);
	filter.filter(*result_cloud);

	visualization::PCLVisualizer viewer("Cloud Viewer");     //创建viewer对象
	viewer.addCoordinateSystem(1.0);
	int v1(0), v2(1);
	
	// 原始点云
	viewer.createViewPort(0, 0, 0.5, 1, v1);	// 对角线坐标（x1,y1,x2,y2）
	viewer.setBackgroundColor(0, 0, 0, v1);
	visualization::PointCloudColorHandlerCustom<PointXYZ> white1(source_cloud, 255, 255, 255);
	viewer.addPointCloud(source_cloud, white1, "source cloud", v1);	// 为点云自定义颜色为白色
	cout << source_cloud->points.size() << endl;

	// 体素化后
	viewer.createViewPort(0.5, 0, 1, 1, v2);
	viewer.setBackgroundColor(0, 0, 0, v2);
	visualization::PointCloudColorHandlerCustom<PointXYZ> white2(result_cloud, 255, 255, 255);
	viewer.addPointCloud(result_cloud, white2, "result_cloud", v2);	// 为点云自定义颜色为白色
	cout << result_cloud->points.size() << endl;
	
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}	// 使窗口停留

	return result_cloud;
}