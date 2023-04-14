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
using namespace pcl;
using std::cout;
using std::endl;

myPointXYZ::Ptr transformByExtrinsics(myPointXYZ::Ptr target_cloud) {
	/* 点云变换，按照相机位姿 */
	Eigen::Matrix4d rotation;
	rotation << 0.999997, 0.00235355, -0.000991121, -32.072,
		-0.00224903, 0.995497, 0.0947632, -2.03661,
		0.00120969, -0.0947606, 0.995499, 3.74932,
		0.0, 0.0, 0.0, 1.0;// 旋转矩阵

	myPointXYZ::Ptr target_cloud_trans(new myPointXYZ);
	transformPointCloud(*target_cloud, *target_cloud_trans, rotation);
	return target_cloud_trans;
}

myPointXYZ::Ptr removeBackground(myPointXYZ::Ptr target_cloud, myPointXYZ::Ptr source_cloud) {
	/* 点云相减 */
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
	mls.setSearchRadius(10); // 拟合半径
	mls.setPolynomialFit(true); // 利用多项式
	mls.setPolynomialFit(3); // 三阶
	mls.setSearchMethod(kdtree);
	mls.process(*target_cloud_smooth);

	return target_cloud_smooth;

}