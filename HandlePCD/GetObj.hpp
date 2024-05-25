#pragma once
#include "GetFeature.hpp"
#include "GetObj.h"
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include <pcl/kdtree/kdtree.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>

template<typename PT>
void myVisualization(typename PointCloud<PT>::Ptr cloud, const char* window_name) {
	visualization::PCLVisualizer viewer(window_name);
	viewer.setBackgroundColor(0, 0, 0); // 设置背景色,RGB,0~1

	// 定义变换矩阵
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0.0, 0.0, -0.0; // 在Z轴上向负方向平移1个单位

	viewer.addCoordinateSystem(1000.0, transform);
	visualization::PointCloudColorHandlerCustom<PT> white(cloud, 255, 255, 255);
	viewer.addPointCloud<PT>(cloud, white, "cloud"); // 显示点云，其中fildColor为颜色显示
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
}

template<typename PT>
void myVisualization2(typename PointCloud<PT>::Ptr cloud1, typename PointCloud<PT>::Ptr cloud2, const char* window_name) {
	visualization::PCLVisualizer viewer(window_name);
	viewer.setBackgroundColor(0, 0, 0); // 设置背景色,RGB,0~1

	// 定义变换矩阵
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0.0, 0.0, -0.0; // 在Z轴上向负方向平移1个单位

	viewer.addCoordinateSystem(1000.0, transform);
	visualization::PointCloudColorHandlerCustom<PT> white(cloud1, 255, 255, 255);
	viewer.addPointCloud<PT>(cloud1, white, "cloud1"); // 显示点云，其中fildColor为颜色显示
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");

	visualization::PointCloudColorHandlerCustom<PT> red(cloud2, 255, 0, 0);
	viewer.addPointCloud<PT>(cloud2, red, "cloud2");
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
}

template<typename PT>
void myVisualization2(typename PointCloud<PT>::Ptr cloud1, typename PointCloud<PointNormal>::Ptr cloud2, const char* window_name) {
	visualization::PCLVisualizer viewer(window_name);
	viewer.setBackgroundColor(0.5, 0.5, 0.5); // 设置背景色,RGB,0~1

	// 定义变换矩阵
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0.0, 0.0, -0.0; // 在Z轴上向负方向平移1个单位

	viewer.addCoordinateSystem(1000.0, transform);
	visualization::PointCloudColorHandlerCustom<PT> white(cloud1, 255, 255, 255);
	viewer.addPointCloud<PT>(cloud1, white, "cloud1"); // 显示点云，其中fildColor为颜色显示
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");

	visualization::PointCloudColorHandlerCustom<PointNormal> red(cloud2, 255, 0, 0);
	viewer.addPointCloud<PointNormal>(cloud2, red, "cloud2");
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
}

template<typename PT>
PointCloud<PT> downSampleForce(typename PointCloud<PT>::Ptr source_cloud, double scale) {
	PointCloud<PT> result_cloud;

	// 随机下采样点云
	RandomSample<PT> filter;
	filter.setInputCloud(source_cloud);
	filter.setSample(source_cloud->points.size() * scale);
	filter.setSeed(3407);
	filter.filter(result_cloud);


	//// 体素下采样
	//VoxelGrid<PT> filter;
	//filter.setInputCloud(source_cloud);
	//filter.setLeafSize(10, 10, 10);
	//filter.filter(result_cloud);

	return result_cloud;
}

template<typename PT>
void removeNoise(typename PointCloud<PT>::Ptr source_cloud) {
	StatisticalOutlierRemoval<PT> sor;
	sor.setInputCloud(source_cloud);
	sor.setMeanK(100);
	sor.setStddevMulThresh(1.0);
	sor.filter(*source_cloud);
}

template<typename PT>
void removeOtherObj(typename PointCloud<PT>::Ptr source_cloud) {
	// 通过下采样的点云进行快速分割，然后将分割结果的包围盒应用到原始点云
	typename PointCloud<PT>::Ptr down_cloud(new PointCloud<PT>);
	*down_cloud = downSampleForce<PT>(source_cloud, 0.05);

	typename search::KdTree<PT>::Ptr tree(new search::KdTree<PT>);
	tree->setInputCloud(down_cloud);

	int num_points = down_cloud->points.size();

	EuclideanClusterExtraction<PT> ec;
	ec.setClusterTolerance(50); // 设置欧式聚类的容差
	ec.setMinClusterSize(0);   // 设置最小的聚类大小
	ec.setMaxClusterSize(num_points);  // 设置最大的聚类大小
	ec.setSearchMethod(tree);
	ec.setInputCloud(down_cloud);

	std::vector<PointIndices> cluster_indices;
	ec.extract(cluster_indices);

	typename PointCloud<PT>::Ptr maxCluster(new PointCloud<PT>);

	std::sort(cluster_indices.begin(), cluster_indices.end(), [](PointIndices a, PointIndices b) {
		return a.indices.size() > b.indices.size();
		});

	copyPointCloud(*down_cloud, cluster_indices[0], *maxCluster);

	PT min{}, max{};
	getMinMax3D(*maxCluster, min, max);
	PassThrough<PT> pass;
	double delta_x = (max.x - min.x) * 0.1;
	pass.setFilterFieldName("x");
	pass.setFilterLimits(min.x - delta_x, max.x + delta_x);
	pass.setInputCloud(source_cloud);
	pass.filter(*source_cloud);

	double delta_y = (max.y - min.y) * 0.1;
	pass.setFilterFieldName("y");
	pass.setFilterLimits(min.y - delta_y, max.y + delta_y);
	pass.setInputCloud(source_cloud);
	pass.filter(*source_cloud);

	double delta_z = (max.z - min.z) * 0.1;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(min.z - delta_z, max.z + delta_z);
	pass.setInputCloud(source_cloud);
	pass.filter(*source_cloud);

	//return maxCluster;
}

template<typename PT>
void smoothByMLS(typename PointCloud<PT>::Ptr source_cloud, typename PointCloud<PointNormal>::Ptr cloud_normal) {
	typename search::KdTree<PT>::Ptr kdtree;
	MovingLeastSquares<PT, PointNormal> mls;
	mls.setInputCloud(source_cloud);
	mls.setSearchRadius(20); // 拟合半径
	mls.setPolynomialOrder(true); // 利用多项式
	mls.setPolynomialOrder(3); // 三阶
	mls.setSearchMethod(kdtree);
	mls.process(*cloud_normal);
}

template<typename PT>
void projectionCloud(typename PointCloud<PT>::Ptr source_cloud, typename PointCloud<PT>::Ptr project_cloud, int des_axis) {
	// 0:x=0, 1:y=0, 2:z=0
	switch (des_axis)
	{
	case 0:
		for (auto& point : *source_cloud) {

			point.x = 0;
			project_cloud->push_back(point);
		}
		break;
	case 1:
		for (auto& point : *source_cloud) {

			point.y = 0;
			project_cloud->push_back(point);
		}
		break;
	case 2:
		for (auto& point : *source_cloud) {

			point.z = 0;
			project_cloud->push_back(point);
		}
		break;
	}
}