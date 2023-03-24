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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	char path[256] = "../../PCD/innerBackground.pcd";
	if (-1 == pcl::io::loadPCDFile(path, *cloud)) {
		cout << "error input!" << endl;
		return -1;
	}

	/* ���ӻ����� */
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);	// ��������

	cout << centroid << endl;
	cout << cloud->points.size() << endl;	// ͳ�Ƶ������


	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");     //����viewer����

	viewer.addCoordinateSystem(3.0, centroid[0], centroid[1], centroid[2], "centroid", 0);	// ������Ϊԭ�㽨ϵ

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 255, 255, 255); 
	viewer.addPointCloud(cloud, rgb, "sample cloud");	// Ϊ�����Զ�����ɫΪ��ɫ

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}	// ʹ����ͣ��

	return 0;
}