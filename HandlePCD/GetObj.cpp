#define myPointXYZ pcl::PointCloud<pcl::PointXYZ>
#define myPointNormal pcl::PointCloud<pcl::PointNormal>
#include "GetObj.h"
#include <iostream>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
using namespace pcl;
using std::cout;
using std::endl;

myPointXYZ::Ptr transformByExtrinsics(myPointXYZ::Ptr target_cloud) {
	/* ���Ʊ任���������λ�� */
	Eigen::Matrix4d rotation;
	rotation << 0.999997, 0.00235355, -0.000991121, -32.072,
		-0.00224903, 0.995497, 0.0947632, -2.03661,
		0.00120969, -0.0947606, 0.995499, 3.74932,
		0.0, 0.0, 0.0, 1.0;// ��ת����

	myPointXYZ::Ptr target_cloud_trans(new myPointXYZ);
	transformPointCloud(*target_cloud, *target_cloud_trans, rotation);
	return target_cloud_trans;
}

myPointXYZ::Ptr removeBackground(myPointXYZ::Ptr target_cloud, myPointXYZ::Ptr source_cloud) {
	/* ������� */
	float resolution = 128.0f;
	myPointXYZ::Ptr source_cloud_result(new myPointXYZ);
	octree::OctreePointCloudChangeDetector<PointXYZ> octree(resolution);
	PointIndices::Ptr inliers(new PointIndices());

	octree.setInputCloud(target_cloud);
	octree.addPointsFromInputCloud();
	octree.switchBuffers();
	octree.setInputCloud(source_cloud);
	octree.addPointsFromInputCloud();

	std::vector<int> newPointIndex;
	octree.getPointIndicesFromNewVoxels(newPointIndex);

	cout << newPointIndex.size() << endl;

	inliers->indices = newPointIndex;

	ExtractIndices<PointXYZ> extract;
	extract.setInputCloud(source_cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*source_cloud_result);
	return source_cloud_result;
}

myPointXYZ::Ptr removeNoise(myPointXYZ::Ptr target_cloud) {
	myPointXYZ::Ptr target_cloud_denoise(new myPointXYZ);

	StatisticalOutlierRemoval<PointXYZ> sor;
	sor.setInputCloud(target_cloud);
	sor.setMeanK(
		50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*target_cloud_denoise);
	return target_cloud_denoise;
}

myPointNormal::Ptr smoothByMLS(myPointXYZ::Ptr target_cloud) {
	myPointNormal::Ptr target_cloud_smooth(new myPointNormal);
	search::KdTree<PointXYZ>::Ptr kdtree;
	MovingLeastSquares<PointXYZ, PointNormal> mls;
	mls.setInputCloud(target_cloud);
	mls.setSearchRadius(10); // ��ϰ뾶
	mls.setPolynomialFit(true); // ���ö���ʽ
	mls.setPolynomialFit(3); // ����
	mls.setSearchMethod(kdtree);
	mls.process(*target_cloud_smooth);

	return target_cloud_smooth;

}

myPointXYZ::Ptr getObj(myPointXYZ::Ptr target_cloud, myPointXYZ::Ptr source_cloud) {
	// ���ӻ�����
	visualization::PCLVisualizer viewer("Cloud Viewer");     //����viewer����
	viewer.addCoordinateSystem();

	int v1(0), v2(1), v3(2);	// �������Ҵ���

	// Ŀ����ƣ�����
	viewer.createViewPort(0, 0, 0.5, 1, v1);	// �Խ������꣨x1,y1,x2,y2��
	viewer.setBackgroundColor(0, 0, 0, v1);
	visualization::PointCloudColorHandlerCustom<PointXYZ> white(target_cloud, 255, 255, 255);
	viewer.addPointCloud(target_cloud, white, "target cloud", v1);	// Ϊ�����Զ�����ɫΪ��ɫ

	// ��������ƣ�����+����
	visualization::PointCloudColorHandlerCustom<PointXYZ> blue(source_cloud, 0, 0, 255);
	viewer.addPointCloud(source_cloud, blue, "source cloud", v1);

	// ����octree��׼��ɾ���󲿷��ظ�����
	myPointXYZ::Ptr source_cloud_result(new myPointXYZ);
	source_cloud_result = removeBackground(target_cloud, source_cloud);

	// ȥ�������ƣ�����+������ɢ��
	viewer.createViewPort(0.5, 0, 1, 1, v2);
	viewer.setBackgroundColor(0, 0, 0, v2);
	//visualization::PointCloudColorHandlerCustom<PointXYZ> white2(source_cloud_result, 255, 255, 255);
	//viewer.addPointCloud(source_cloud_result, white2, "source cloud result", v2);

	// StatisticalOutlierRemoval�˲�
	myPointXYZ::Ptr source_cloud_denoise(new myPointXYZ);
	source_cloud_denoise = removeNoise(source_cloud_result);

	// �˲�����ƣ�����
	visualization::PointCloudColorHandlerCustom<PointXYZ> white2(source_cloud_denoise, 255, 255, 255);
	viewer.addPointCloud(source_cloud_denoise, white2, "source cloud denoise", v2);

	//// MovingLeastSquares �ϲ�����ƽ������
	//myPointNormal::Ptr source_cloud_smooth(new myPointNormal);
	//source_cloud_smooth = smoothByMLS(source_cloud_denoise);

	//// ƽ������ƣ�����
	//viewer.createViewPort(0.5, 0, 1, 0.5, v3);
	//viewer.setBackgroundColor(0, 0, 0, v3);
	//visualization::PointCloudColorHandlerCustom<PointNormal> white3(source_cloud_smooth, 255, 255, 255);
	//viewer.addPointCloud(source_cloud_smooth, white3, "source cloud smooth", v3);

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}	// ʹ����ͣ��

	return source_cloud_denoise;
}