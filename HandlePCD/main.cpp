#include<pcl/visualization/pcl_visualizer.h>
#include<iostream>//标准C++库中的输入输出类相关头文件。
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。
#include<Eigen/Core>
#include <pcl/common/transforms.h>

int user_data;
using std::cout;

int main() {
	/* 导入点云 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	char target[256] = "../PCD/innerBackground.pcd";
	char source[256] = "../PCD/0321-1.pcd";
	if (-1 == pcl::io::loadPCDFile(target, *targetCloud)) {
		cout << "error input!" << endl;
		return -1;
	}
	if (-1 == pcl::io::loadPCDFile(source, *sourceCloud)) {
		cout << "error input!" << endl;
		return -1;
	}
	cout << targetCloud->points.size() << endl;	// 统计点的数量
	cout << sourceCloud->points.size() << endl;	// 统计点的数量


	/* 可视化点云 */
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");     //创建viewer对象

	int v1(0), v2(1), v3(2), v4(3);	// 定义左右窗口

	viewer.createViewPort(0, 0.5, 0.5, 1.0, v1);	// 对角线坐标（x1,y1,x2,y2）
	viewer.setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb1(targetCloud, 255, 255, 255);
	viewer.addPointCloud(targetCloud, rgb1, "target cloud", v1);	// 为点云自定义颜色为白色

	viewer.createViewPort(0.5, 0.5, 1, 1, v2);
	viewer.setBackgroundColor(0, 0, 0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb2(sourceCloud, 255, 255, 255);
	viewer.addPointCloud(sourceCloud, rgb2, "source cloud", v2);

	/* 点云变换，按照相机位姿 */
	Eigen::Matrix4d rotation;
	rotation << 0.999997, 0.00235355, -0.000991121, -32.072,
		-0.00224903, 0.995497, 0.0947632, -2.03661,
		0.00120969, -0.0947606, 0.995499, 3.74932,
		0.0, 0.0, 0.0, 1.0;// 旋转矩阵


	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*sourceCloud, *sourceCloudOut, rotation);

	viewer.createViewPort(0, 0, 0.5, 0.5, v3);
	viewer.setBackgroundColor(0, 0, 0, v3);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb3(sourceCloudOut, 255, 255, 255);
	viewer.addPointCloud(sourceCloudOut, rgb3, "source cloud out", v3);

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}	// 使窗口停留

	return 0;
}