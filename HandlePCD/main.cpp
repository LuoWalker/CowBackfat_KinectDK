#define myPointXYZ pcl::PointCloud<pcl::PointXYZ>
#define myPointNormal pcl::PointCloud<pcl::PointNormal>
#include "GetObj.h"
#include "GetFeature.h"
#include <iostream> //��׼C++���е�������������ͷ�ļ���
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h> //pcd ��д����ص�ͷ�ļ���
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���
#include <filesystem> // �����ļ�
#include <direct.h>

using namespace std;
using std::experimental::filesystem::v1::directory_iterator;
using std::experimental::filesystem::v1::path;

int main() {
	/* ������� */
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

			cout << "��ʼ���ƣ�" << source_cloud->points.size() << endl;	// ͳ�Ƶ������

			source_cloud = downSampleVoxelization(source_cloud);
			cout << "�²������ػ���" << source_cloud->points.size() << endl;

			/* ���ȥ���������˲���ĵ��� */
			result_cloud = getObj(target_cloud, source_cloud);

			result_cloud = limitArea(result_cloud, filename);
			cout << "���Ƹ߶Ⱥ�" << result_cloud->points.size() << endl;

			///* ���ػ� */
			//if (-1 == pcl::io::loadPCDFile("../PCD/objectTest.pcd", *result_cloud)) {
			//	cout << "error input!" << endl;
			//	return -1;
			//}

			/* ������ά͹�� */
			//result_cloud = getConvexHull(result_cloud);

			/* ������� */
			pcl::io::savePCDFileASCII(object_pcd_path + filename, *result_cloud);

		}
	}

	return 0;
}