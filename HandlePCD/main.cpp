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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	char path[256] = "../../PCD/innerBackground.pcd";
	if (-1 == pcl::io::loadPCDFile(path, *cloud)) {
		cout << "error input!" << endl;
		return -1;
	}

	/* 可视化点云 */
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);	// 计算质心

	cout << centroid << endl;
	cout << cloud->points.size() << endl;	// 统计点的数量


	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");     //创建viewer对象

	viewer.addCoordinateSystem(3.0, centroid[0], centroid[1], centroid[2], "centroid", 0);	// 以质心为原点建系

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 255, 255, 255); 
	viewer.addPointCloud(cloud, rgb, "sample cloud");	// 为点云自定义颜色为白色

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}	// 使窗口停留

	return 0;
}