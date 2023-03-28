#include "GetObj.h"
#include<pcl/visualization/pcl_visualizer.h>
#include<iostream>//标准C++库中的输入输出类相关头文件。
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。



int user_data;
using std::cout;

int main() {
	/* 导入点云 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	char target[256] = "../PCD/innerBackground.pcd";
	char source[256] = "../PCD/0321-1.pcd";
	if (-1 == pcl::io::loadPCDFile(target, *target_cloud)) {
		cout << "error input!" << endl;
		return -1;
	}
	if (-1 == pcl::io::loadPCDFile(source, *source_cloud)) {
		cout << "error input!" << endl;
		return -1;
	}
	cout << target_cloud->points.size() << endl;	// 统计点的数量
	cout << source_cloud->points.size() << endl;	// 统计点的数量


	// 可视化点云
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");     //创建viewer对象
	viewer.addCoordinateSystem();

	int v1(0), v2(1);	// 定义左右窗口

	// 目标点云，背景
	viewer.createViewPort(0, 0, 0.5, 1, v1);	// 对角线坐标（x1,y1,x2,y2）
	viewer.setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(target_cloud, 255, 255, 255);
	viewer.addPointCloud(target_cloud, white, "target cloud", v1);	// 为点云自定义颜色为白色

	// 待处理点云，物体+背景
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(source_cloud, 0, 0, 255);
	viewer.addPointCloud(source_cloud, blue, "source cloud", v1);

	// 利用octree配准，删除大部分重复点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_result(new pcl::PointCloud<pcl::PointXYZ>);
	source_cloud_result = removeBackground(target_cloud, source_cloud);

	// 处理后点云，物体+部分离散点
	viewer.createViewPort(0.5, 0, 1, 1, v2);
	viewer.setBackgroundColor(0, 0, 0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white2(source_cloud_result, 255, 255, 255);
	viewer.addPointCloud(source_cloud_result, white2, "source cloud result", v2);

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}	// 使窗口停留

	return 0;
}