#include<pcl/visualization/pcl_visualizer.h>
#include<iostream>//��׼C++���е�������������ͷ�ļ���
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���
#include<Eigen/Core>
#include <pcl/common/transforms.h>

int user_data;
using std::cout;

int main() {
	/* ������� */
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
	cout << targetCloud->points.size() << endl;	// ͳ�Ƶ������
	cout << sourceCloud->points.size() << endl;	// ͳ�Ƶ������


	/* ���ӻ����� */
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");     //����viewer����

	int v1(0), v2(1), v3(2), v4(3);	// �������Ҵ���

	viewer.createViewPort(0, 0.5, 0.5, 1.0, v1);	// �Խ������꣨x1,y1,x2,y2��
	viewer.setBackgroundColor(0, 0, 0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb1(targetCloud, 255, 255, 255);
	viewer.addPointCloud(targetCloud, rgb1, "target cloud", v1);	// Ϊ�����Զ�����ɫΪ��ɫ

	viewer.createViewPort(0.5, 0.5, 1, 1, v2);
	viewer.setBackgroundColor(0, 0, 0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb2(sourceCloud, 255, 255, 255);
	viewer.addPointCloud(sourceCloud, rgb2, "source cloud", v2);

	/* ���Ʊ任���������λ�� */
	Eigen::Matrix4d rotation;
	rotation << 0.999997, 0.00235355, -0.000991121, -32.072,
		-0.00224903, 0.995497, 0.0947632, -2.03661,
		0.00120969, -0.0947606, 0.995499, 3.74932,
		0.0, 0.0, 0.0, 1.0;// ��ת����


	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*sourceCloud, *sourceCloudOut, rotation);

	viewer.createViewPort(0, 0, 0.5, 0.5, v3);
	viewer.setBackgroundColor(0, 0, 0, v3);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb3(sourceCloudOut, 255, 255, 255);
	viewer.addPointCloud(sourceCloudOut, rgb3, "source cloud out", v3);

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}	// ʹ����ͣ��

	return 0;
}