#include "GetObj.h"
#include<pcl/visualization/pcl_visualizer.h>
#include<iostream>//��׼C++���е�������������ͷ�ļ���
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
#include<pcl/io/ply_io.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���



int user_data;
using std::cout;

int main() {
	/* ������� */
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
	cout << target_cloud->points.size() << endl;	// ͳ�Ƶ������
	cout << source_cloud->points.size() << endl;	// ͳ�Ƶ������


	// ���ӻ�����
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");     //����viewer����
	viewer.addCoordinateSystem();

	int v1(0), v2(1);	// �������Ҵ���

	// Ŀ����ƣ�����
	viewer.createViewPort(0, 0, 0.5, 1, v1);	// �Խ������꣨x1,y1,x2,y2��
	viewer.setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(target_cloud, 255, 255, 255);
	viewer.addPointCloud(target_cloud, white, "target cloud", v1);	// Ϊ�����Զ�����ɫΪ��ɫ

	// ��������ƣ�����+����
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(source_cloud, 0, 0, 255);
	viewer.addPointCloud(source_cloud, blue, "source cloud", v1);

	// ����octree��׼��ɾ���󲿷��ظ�����
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_result(new pcl::PointCloud<pcl::PointXYZ>);
	source_cloud_result = removeBackground(target_cloud, source_cloud);

	// �������ƣ�����+������ɢ��
	viewer.createViewPort(0.5, 0, 1, 1, v2);
	viewer.setBackgroundColor(0, 0, 0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white2(source_cloud_result, 255, 255, 255);
	viewer.addPointCloud(source_cloud_result, white2, "source cloud result", v2);

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}	// ʹ����ͣ��

	return 0;
}