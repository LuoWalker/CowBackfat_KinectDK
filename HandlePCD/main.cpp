#define myPointXYZ pcl::PointCloud<pcl::PointXYZ>
#define myPointNormal pcl::PointCloud<pcl::PointNormal>
#include "GetObj.h"
#include "GetFeature.h"
#include <iostream>//标准C++库中的输入输出类相关头文件。
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> //PCL中支持的点类型头文件。
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。

using namespace std;

int main() {
	///* 导入点云 */
	//myPointXYZ::Ptr target_cloud(new myPointXYZ);
	//myPointXYZ::Ptr source_cloud(new myPointXYZ);
	myPointXYZ::Ptr result_cloud(new myPointXYZ);
	//char target[256] = "../PCD/innerBackground.pcd";
	//char source[256] = "../PCD/0321-1.pcd";
	//if (-1 == pcl::io::loadPCDFile(target, *target_cloud)) {
	//	cout << "error input!" << endl;
	//	return -1;
	//}
	//if (-1 == pcl::io::loadPCDFile(source, *source_cloud)) {
	//	cout << "error input!" << endl;
	//	return -1;
	//}
	////cout << target_cloud->points.size() << endl;	// 统计点的数量
	////cout << source_cloud->points.size() << endl;	// 统计点的数量
	////cout << *source_cloud << endl;

	///* 获得去除背景、滤波后的点云 */
	//result_cloud =getObj(target_cloud, source_cloud);

	/* 体素化 */
	if (-1 == pcl::io::loadPCDFile("../PCD/objectTest.pcd", *result_cloud)) {
		cout << "error input!" << endl;
		return -1;
	}
	downSampleVoxelization(result_cloud);


	/* 保存点云 */
	//pcl::io::savePCDFileASCII("../PCD/objectTest.pcd", *result_cloud); 

	return 0;
}