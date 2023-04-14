#define myPointXYZ pcl::PointCloud<pcl::PointXYZ>
#define myPointNormal pcl::PointCloud<pcl::PointNormal>
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
	myPointXYZ::Ptr target_cloud(new myPointXYZ);
	myPointXYZ::Ptr source_cloud(new myPointXYZ);
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

	int v1(0), v2(1), v3(2);	// �������Ҵ���

	// Ŀ����ƣ�����
	viewer.createViewPort(0, 0, 0.5, 1, v1);	// �Խ������꣨x1,y1,x2,y2��
	viewer.setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(target_cloud, 255, 255, 255);
	viewer.addPointCloud(target_cloud, white, "target cloud", v1);	// Ϊ�����Զ�����ɫΪ��ɫ

	// ��������ƣ�����+����
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(source_cloud, 0, 0, 255);
	viewer.addPointCloud(source_cloud, blue, "source cloud", v1);

	// ����octree��׼��ɾ���󲿷��ظ�����
	myPointXYZ::Ptr source_cloud_result(new myPointXYZ);
	source_cloud_result = removeBackground(target_cloud, source_cloud);

	// ȥ�������ƣ�����+������ɢ��
	viewer.createViewPort(0.5, 0, 1, 1, v2);
	viewer.setBackgroundColor(0, 0, 0, v2);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white2(source_cloud_result, 255, 255, 255);
	//viewer.addPointCloud(source_cloud_result, white2, "source cloud result", v2);

	// StatisticalOutlierRemoval�˲�
	myPointXYZ::Ptr source_cloud_denoise(new myPointXYZ);
	source_cloud_denoise = removeNoise(source_cloud_result);

	// �˲�����ƣ�����
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white2(source_cloud_denoise, 255, 255, 255);
	viewer.addPointCloud(source_cloud_denoise, white2, "source cloud denoise", v2);

	//// MovingLeastSquares �ϲ�����ƽ������
	//myPointNormal::Ptr source_cloud_smooth(new myPointNormal);
	//source_cloud_smooth = smoothByMLS(source_cloud_denoise);

	//// ƽ������ƣ�����
	//viewer.createViewPort(0.5, 0, 1, 0.5, v3);
	//viewer.setBackgroundColor(0, 0, 0, v3);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> white3(source_cloud_smooth, 255, 255, 255);
	//viewer.addPointCloud(source_cloud_smooth, white3, "source cloud smooth", v3);

	// �������
	pcl::io::savePCDFileASCII("../PCD/objectTest.pcd", *source_cloud_denoise);

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}	// ʹ����ͣ��

	return 0;
}