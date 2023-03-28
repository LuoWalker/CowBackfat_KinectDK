#include "GetObj.h"
#include <iostream>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/common/transforms.h>
using namespace pcl;
using std::cout;
using std::endl;

PointCloud<PointXYZ>::Ptr removeBackground(PointCloud<PointXYZ>::Ptr target_cloud, PointCloud<PointXYZ>::Ptr source_cloud) {
	/* 点云相减 */
	float resolution = 32.0f;
	PointCloud<PointXYZ>::Ptr source_cloud_result(new PointCloud<PointXYZ>);
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

PointCloud<PointXYZ>::Ptr transformByExtrinsics(PointCloud<PointXYZ>::Ptr target_cloud) {
	/* 点云变换，按照相机位姿 */
	Eigen::Matrix4d rotation;
	rotation << 0.999997, 0.00235355, -0.000991121, -32.072,
		-0.00224903, 0.995497, 0.0947632, -2.03661,
		0.00120969, -0.0947606, 0.995499, 3.74932,
		0.0, 0.0, 0.0, 1.0;// 旋转矩阵

	PointCloud<PointXYZ>::Ptr target_cloud_trans(new PointCloud<PointXYZ>);
	transformPointCloud(*target_cloud, *target_cloud_trans, rotation);
	return target_cloud_trans;
}