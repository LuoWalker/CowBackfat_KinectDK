#define myPointXYZ pcl::PointCloud<pcl::PointXYZ>
#define myPointNormal pcl::PointCloud<pcl::PointNormal>
#include "GetObj.h"
#include "GetFeature.h"
#include <iostream> //标准C++库中的输入输出类相关头文件。
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h> //pcd 读写类相关的头文件。
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> //PCL中支持的点类型头文件。
#include <filesystem> // 遍历文件
#include <direct.h>

using namespace std;
using std::experimental::filesystem::v1::directory_iterator;
using std::experimental::filesystem::v1::path;

int main() {
	/* 导入点云 */
	myPointXYZ::Ptr target_cloud(new myPointXYZ);
	myPointXYZ::Ptr source_cloud(new myPointXYZ);
	myPointXYZ::Ptr result_cloud(new myPointXYZ);

	cout << "target" << endl;
	string target = "../PCD/innerBackground.pcd";
	pcl::io::loadPCDFile(target, *target_cloud);

	for (size_t i = 1; i <= 9; i++)
	{
		if (i == 8 || i == 9) {
			string target = "../PCD/outerBackground.pcd";
			pcl::io::loadPCDFile(target, *target_cloud);
		}
		string dir_name = "0321" + to_string(i) + "/";
		string origin_pcd_path = "../PCD/origin/" + dir_name;
		string object_pcd_path = "../PCD/object/" + dir_name;
		cout << origin_pcd_path << endl;

		if (0 != _access(object_pcd_path.c_str(), 0)) {
			_mkdir(object_pcd_path.c_str());
		}
		for (auto &v : directory_iterator(origin_pcd_path))
		{
			string filename = v.path().filename().string();

			string source = origin_pcd_path + filename;
			cout << source << endl;
			pcl::io::loadPCDFile(source, *source_cloud);

			cout << "初始点云：" << source_cloud->points.size() << endl;	// 统计点的数量

			source_cloud = downSampleVoxelization(source_cloud);
			cout << "下采样体素化后：" << source_cloud->points.size() << endl;

			/* 获得去除背景、滤波后的点云 */
			result_cloud = getObj(target_cloud, source_cloud);

			result_cloud = limitArea(result_cloud, filename);
			cout << "限制高度后：" << result_cloud->points.size() << endl;

			///* 体素化 */
			//if (-1 == pcl::io::loadPCDFile("../PCD/objectTest.pcd", *result_cloud)) {
			//	cout << "error input!" << endl;
			//	return -1;
			//}

			/* 计算三维凸包 */
			//result_cloud = getConvexHull(result_cloud);

			/* 保存点云 */
			pcl::io::savePCDFileASCII(object_pcd_path + filename, *result_cloud);

		}
	}

	return 0;
}