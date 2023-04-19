#define myPointXYZ pcl::PointCloud<pcl::PointXYZ>
#define myPointNormal pcl::PointCloud<pcl::PointNormal>
#include "GetObj.h"
#include "GetFeature.h"
#include <iostream>//��׼C++���е�������������ͷ�ļ���
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���

using namespace std;

int main() {
	///* ������� */
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
	////cout << target_cloud->points.size() << endl;	// ͳ�Ƶ������
	////cout << source_cloud->points.size() << endl;	// ͳ�Ƶ������
	////cout << *source_cloud << endl;

	///* ���ȥ���������˲���ĵ��� */
	//result_cloud =getObj(target_cloud, source_cloud);

	/* ���ػ� */
	if (-1 == pcl::io::loadPCDFile("../PCD/objectTest.pcd", *result_cloud)) {
		cout << "error input!" << endl;
		return -1;
	}
	downSampleVoxelization(result_cloud);


	/* ������� */
	//pcl::io::savePCDFileASCII("../PCD/objectTest.pcd", *result_cloud); 

	return 0;
}