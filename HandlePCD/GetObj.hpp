#pragma once
#include "nanoflann.hpp"
#include "utils.h"
#include <boost/container_hash/is_contiguous_range.hpp>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h>
#include <Eigen/src/Geometry/Transform.h>
#include <limits>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/filters/random_sample.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <vector>
#include "DBSCAN_kdtree.h"
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkSmartPointer.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkRenderWindowInteractor.h>
using namespace pcl;

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
	transform.translation() << 0.0, 0.0, 0.0;

	viewer.addCoordinateSystem(1000.0, transform);
	visualization::PointCloudColorHandlerCustom<PT> white(cloud1, 255, 255, 255);
	viewer.addPointCloud<PT>(cloud1, white, "cloud1"); // 显示点云，其中fildColor为颜色显示
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");

	visualization::PointCloudColorHandlerCustom<PT> red(cloud2, 255, 0, 0);
	viewer.addPointCloud<PT>(cloud2, red, "cloud2");
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");

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

	//// 随机下采样点云
	//RandomSample<PT> filter;
	//filter.setInputCloud(source_cloud);
	//filter.setSample(source_cloud->points.size() * scale);
	//filter.setSeed(3407);
	//filter.filter(result_cloud);

	//// 均匀下采样
	//UniformSampling<PointXYZ> filter;
	//filter.setInputCloud(source_cloud);
	//filter.setRadiusSearch(10);
	//filter.filter(result_cloud);

	// 体素下采样
	VoxelGrid<PT> filter;
	filter.setInputCloud(source_cloud);
	filter.setLeafSize(10, 10, 10);
	filter.filter(result_cloud);

	return result_cloud;
}

template<typename PT>
PointCloud<PT> randomSampleForce(typename PointCloud<PT>::Ptr source_cloud, double scale) {
	PointCloud<PT> result_cloud;

	// 随机下采样点云
	RandomSample<PT> filter;
	filter.setInputCloud(source_cloud);
	filter.setSample(source_cloud->points.size() * scale);
	filter.setSeed(3407);
	filter.filter(result_cloud);

	return result_cloud;
}

template<typename PT>
std::vector<double> getNormalChangeRate(typename PointCloud<PT>::Ptr source_cloud, int search_radius) {
	// 初始化KD树
	KdTreeFLANN<PT> kdtree;
	kdtree.setInputCloud(source_cloud);
	PointIndices::Ptr inliers(new PointIndices());
	std::vector<double> ncr;
	std::vector<int> point_idx_radius_search;
	std::vector<float> point_radius_squared_distance;
	typename PointCloud<PT>::Ptr neighborhood(new PointCloud<PT>);

	//#pragma omp parallel for
	for (int i = 0; i < source_cloud->points.size(); ++i) {
		point_idx_radius_search.clear();
		point_radius_squared_distance.clear();
		if (kdtree.radiusSearch(source_cloud->points[i], search_radius, point_idx_radius_search, point_radius_squared_distance) > 0)
		{
			copyPointCloud(*source_cloud, point_idx_radius_search, *neighborhood);

			Eigen::Vector4f centroid;
			compute3DCentroid(*neighborhood, centroid);

			Eigen::Matrix3f covariance;
			computeCovarianceMatrix(*neighborhood, centroid, covariance);

			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig_solver(covariance);
			if (eig_solver.info() != Eigen::Success)
			{
				ncr.push_back(std::numeric_limits<double>::quiet_NaN());
			}
			Eigen::Vector3f eigenvalues = eig_solver.eigenvalues();

			double sum = eigenvalues[0] + eigenvalues[1] + eigenvalues[2];
			if (sum < std::numeric_limits<double>::epsilon())
			{
				ncr.push_back(std::numeric_limits<double>::quiet_NaN());
			}
			double eMin = std::min(eigenvalues[0], std::min(eigenvalues[1], eigenvalues[2]));
			//#pragma omp critical{
			ncr.push_back(eMin / sum);
			//}
		}
	}
	return ncr;
}

template<typename PT>
std::vector<double> getNormalChangeRate(typename PointCloud<PT>::Ptr target_cloud, typename PointCloud<PT>::Ptr source_cloud, int search_radius) {
	// 初始化KD树
	KdTreeFLANN<PT> kdtree;
	kdtree.setInputCloud(source_cloud);
	PointIndices::Ptr inliers(new PointIndices());
	std::vector<double> ncr;
	std::vector<int> point_idx_radius_search;
	std::vector<float> point_radius_squared_distance;
	typename PointCloud<PT>::Ptr neighborhood(new PointCloud<PT>);

	for (int i = 0; i < target_cloud->points.size(); ++i) {
		point_idx_radius_search.clear();
		point_radius_squared_distance.clear();
		if (kdtree.radiusSearch(target_cloud->points[i], search_radius, point_idx_radius_search, point_radius_squared_distance) > 0)
		{
			copyPointCloud(*source_cloud, point_idx_radius_search, *neighborhood);

			Eigen::Vector4f centroid;
			compute3DCentroid(*neighborhood, centroid);

			Eigen::Matrix3f covariance;
			computeCovarianceMatrix(*neighborhood, centroid, covariance);

			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig_solver(covariance);
			if (eig_solver.info() != Eigen::Success)
			{
				ncr.push_back(std::numeric_limits<double>::quiet_NaN());
			}
			Eigen::Vector3f eigenvalues = eig_solver.eigenvalues();

			double sum = eigenvalues[0] + eigenvalues[1] + eigenvalues[2];
			if (sum < std::numeric_limits<double>::epsilon())
			{
				ncr.push_back(std::numeric_limits<double>::quiet_NaN());
			}
			double eMin = std::min(eigenvalues[0], std::min(eigenvalues[1], eigenvalues[2]));
			ncr.push_back(eMin / sum);
		}
	}
	return ncr;
}

template<typename PT>
void countUpperCornerPoints(typename PointCloud<PT>::Ptr source_cloud, const PT search_point, double radius, int& left_upper_count, int& right_upper_count, int& lower_count) {
	// 创建K-d树用于邻域搜索
	KdTreeFLANN<PT> kdtree;
	kdtree.setInputCloud(source_cloud);

	std::vector<int> point_indices;
	std::vector<float> point_squared_distances;

	// 执行半径搜索
	if (kdtree.radiusSearch(search_point, radius, point_indices, point_squared_distances) > 0) {
		//left_upper_count = 0;
		//right_upper_count = 0;
		//lower_count = 0;

		// 遍历邻域内的点
		for (const int& idx : point_indices) {
			const PT& neighbor_point = source_cloud->points[idx];
			if (neighbor_point.z > search_point.z && neighbor_point.y > search_point.y) {
				// 左上角
				left_upper_count++;
			}
			else if (neighbor_point.z > search_point.z && neighbor_point.y < search_point.y) {
				// 右上角
				right_upper_count++;
			}
			else if (neighbor_point.z < search_point.z) {
				// 下方
				lower_count++;
			}
		}
	}
}

// 判断一个点是否为其邻域内的最低点
template<typename PT>
bool isLowestPointInNeighborhood(typename PointCloud<PT>::Ptr source_cloud, int index, double radius) {
	KdTreeFLANN<PT> kdtree;
	kdtree.setInputCloud(source_cloud);

	PT searchPoint = source_cloud->points[index];
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// 搜索邻域内的点
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
		for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
			if (source_cloud->points[pointIdxRadiusSearch[i]].z < searchPoint.z) {
				return false;
			}
		}
	}

	return true;
}

template<typename PT>
void find_and_remove_min_z_points2(typename PointCloud<PT>::Ptr source_cloud, int direction) {
	// direction: 0 -> 左，删除较大的，1 -> 右，删除较小的
	std::function<bool(int, int)> comparator;
	std::function<int(int, int)> operation;
	int thre_tan;
	if (direction == 0) {
		comparator = [](int a, int b) { return a >= b; }; // 大于
		operation = [](int a, int b) { return a - b; };  // 减法
		thre_tan = 1;
	}
	else
	{
		comparator = [](int a, int b) { return a <= b; };  // 小于
		operation = [](int a, int b) { return a + b; };  // 加法
		thre_tan = -1;
	}

	// 划分区间，对点云进行切片
	PT min{}, max{};
	getMinMax3D(*source_cloud, min, max);
	float cloud_mean = (min.y + max.y) / 2;
	float gap = 10.0;
	int n = (max.x - min.x) / gap + 1;
	std::vector<double> intervals(n);
	for (int i = 0; i < n; ++i) {
		intervals[i] = min.x + gap * i;
	}

	//myVisualization<PT>(source_cloud, "half");
	typename PointCloud<PT>::Ptr min_cloud(new PointCloud<PT>);

	// 遍历点云，找到最小Z值的点
	std::vector<PT> interval_min_z(n, { 0, 0, std::numeric_limits<float>::max() });
	std::vector<PT> interval_min_y(n, { 0, std::numeric_limits<float>::max(), 0 });
	std::vector<PT> interval_max_y(n, { 0, std::numeric_limits<float>::min(), 0 });

	// 提取XY平面上的点
	std::vector<std::vector<Eigen::Vector2d>> interval_points(n);

	//#pragma omp parallel for 
	// 遍历点云
	for (int i = 0; i < source_cloud->points.size(); i++) {
		// 判断在哪个区间
		int n = (source_cloud->points[i].x - min.x) / gap;
		interval_points[n].push_back(Eigen::Vector2d(source_cloud->points[i].y, source_cloud->points[i].z));
		// 更新最小Z值点
		if (source_cloud->points[i].z < interval_min_z[n].z) {
			interval_min_z[n] = source_cloud->points[i];
		}
		if (source_cloud->points[i].y < interval_min_y[n].y) {
			interval_min_y[n] = source_cloud->points[i];
		}
		if (source_cloud->points[i].y > interval_max_y[n].y) {
			interval_max_y[n] = source_cloud->points[i];
		}
	}
	std::vector<int> valid_interval_index;
	for (int i = 0; i < interval_min_z.size(); i++) {
		float width = interval_max_y[i].y - interval_min_y[i].y;
		if (width > (max.y - min.y) * 0.6) {
			if (interval_min_z[i].y > interval_min_y[i].y + 0.3 * width) {
				min_cloud->push_back(interval_min_z[i]);
				valid_interval_index.push_back(i);
			}
		}
	}
	float min_value_sum = 0;
	float min_value_count = 0;
	int count = 0;
	for (int i = 0; i < valid_interval_index.size(); i++) {
		int index = valid_interval_index[i];
		std::vector<Eigen::Vector2d> points = interval_points[index];
		// 拟合多项式
		int degree = 3; // 设置多项式的阶数
		Eigen::VectorXd coefficients;
		fitPolynomial(points, degree, coefficients);
		// 计算多项式的导数
		Eigen::VectorXd derivative = polynomialDerivative(coefficients);
		// 极值点的坐标
		std::vector<double> extrema = findExtrema(derivative, interval_min_y[index].y, interval_max_y[index].y, direction);
		std::vector<double> extrema_z;
		if (extrema.size() == 2) {
			for (auto y : extrema) {
				float z = 0.0;
				for (int i = 0; i <= degree; ++i) {
					z += coefficients[i] * std::pow(y, i);
				}
				extrema_z.push_back(z);
			}
			if (abs(extrema_z[0] - extrema_z[1]) < 70) {
				count += 1;
			}
			else {
				count += 2;
			}
		}
		else {
			count += 1;
		}
		continue;
		// 极值点的信息
		std::cout << "Extrema points: ";
		for (double x : extrema) {
			double second_derivative = evaluateSecondDerivative(coefficients, x);
			if (second_derivative > 0) {
				//std::cout << "Min at x = " << x << " ";
				std::cout << 0;
			}
			else if (second_derivative < 0) {
				//std::cout << "Max at x = " << x << " ";
				std::cout << 1;
			}
		}
		std::cout << std::endl;
		if (evaluateSecondDerivative(coefficients, extrema[1]) < 0) continue;
		//std::vector<double> extrema_z;
		for (auto y : extrema) {
			float z = 0.0;
			for (int i = 0; i <= degree; ++i) {
				z += coefficients[i] * std::pow(y, i);
			}
			extrema_z.push_back(z);
		}
		if (abs(extrema_z[0] - extrema_z[1]) < 70) continue;

		min_value_sum += extrema[1];
		min_value_count++;

		// 可视化点云和拟合曲线
		visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		// 定义变换矩阵
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.translation() << 0.0, 0.0, -0.0; // 在Z轴上向负方向平移1个单位

		viewer->addCoordinateSystem(1000.0, transform);
		viewer->addPointCloud<PointXYZ>(source_cloud, "sample cloud");
		// 可视化拟合的曲线
		PointCloud<PointXYZ>::Ptr curve_cloud(new PointCloud<PointXYZ>);
		for (double y = interval_min_y[index].y; y <= interval_max_y[index].y; y += 0.1) {
			double z = 0;
			for (int i = 0; i <= degree; ++i) {
				z += coefficients[i] * std::pow(y, i);
			}
			curve_cloud->points.emplace_back(intervals[index], y, z);
		}
		visualization::PointCloudColorHandlerCustom<PointXYZ> curve_color_handler(curve_cloud, 255, 0, 0);
		viewer->addPointCloud<PointXYZ>(curve_cloud, curve_color_handler, "curve");
		viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "curve");
		while (!viewer->wasStopped()) {
			viewer->spinOnce(100);
		}
	}
	cout << "ratio:" << count * 1.0 / valid_interval_index.size() << "." << count << "/" << valid_interval_index.size() << endl;
	return;

	// 可视化点云和拟合曲线
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<PointXYZ>(source_cloud, "sample cloud");
	if (min_value_count > (interval_points.size() - 10) * 0.4) {
		float min_value_mean = min_value_sum / min_value_count;
		// 可视化拟合的曲线
		PointCloud<PointXYZ>::Ptr line(new PointCloud<PointXYZ>);
		for (float x = min.x; x <= max.x; x += 0.1) {
			float z = max.z - 50;
			float y = min_value_mean;
			line->points.emplace_back(x, y, z);
		}
		visualization::PointCloudColorHandlerCustom<PointXYZ> curve_color_handler(line, 255, 0, 0);
		viewer->addPointCloud<PointXYZ>(line, curve_color_handler, "line");
		viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "line");
	}
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
	}
	return;
	myVisualization2<PT>(source_cloud, min_cloud, "min_cloud");

	{
		// 根据相邻点Y距离进行分段
		std::vector<float> distances;
		distances.push_back(0);
		std::vector<int> split_index;
		split_index.push_back(0);
		float sum = 0, origin_sum = 0;
		std::vector<float> means;
		for (int i = 1; i < interval_min_z.size(); ++i) {
			origin_sum += interval_min_z[i - 1].y;
			float distance = interval_min_z[i - 1].y - interval_min_z[i].y;
			distances.push_back(distance);
			sum += interval_min_z[i - 1].y;
			if (std::abs(distance) > 100) {
				float mean = sum / (i - split_index[split_index.size() - 1]);
				means.push_back(mean);
				sum = 0; mean = 0;
				split_index.push_back(i);
			}
		}
		sum += interval_min_z[interval_min_z.size() - 1].y;
		split_index.push_back(interval_min_z.size());
		float mean = sum / (interval_min_z.size() - split_index[split_index.size() - 2]);
		means.push_back(mean);

		// 先区分区间，对区间断面进行多项式拟合判断是否为交界
		int left = 0, right = 0;
		std::vector<std::vector<int>> segment_count;
		for (int i = 0; i < split_index.size() - 1; i++) {
			left = split_index[i];
			right = split_index[i + 1] - 1;
			cout << "left: " << left << " right: " << right << endl;

			//segment_count.push_back({ left_upper_count, right_upper_count, lower_count });
		}
	}

	//origin_sum += interval_min_z[interval_min_z.size() - 1].y;
	//float origin_mean = origin_sum / interval_min_z.size();

	//for (int i = 0; i < means.size(); i++) {
	//	if (comparator(means[i], origin_mean)) {
	//		left = split_index[i];
	//		right = split_index[i + 1] - 1;
	//		for (int j = left; j <= right; j++) {
	//			min_cloud->push_back(interval_min_z[j]);
	//			interval_min_z[j].x = 99999; // 标记为有效点
	//		}
	//	}
	//}

	//myVisualization2<PT>(source_cloud, min_cloud, "min_cloud");
	//myVisualization<PT>(min_cloud, "min_cloud");

	PointIndices::Ptr inliers(new PointIndices());
	// 遍历点云
	for (int i = 0; i < source_cloud->points.size(); ++i) {
		// 判断在哪个区间
		int n = (source_cloud->points[i].x - min.x) / gap;
		float delta_y = source_cloud->points[i].y - interval_min_z[n].y;
		float delta_z = source_cloud->points[i].z - interval_min_z[n].z;
		if (interval_min_z[n].x == 99999) {
			if (comparator(source_cloud->points[i].y, operation(interval_min_z[n].y, 10)) && (source_cloud->points[i].z < (interval_min_z[n].z + 50))) {
				inliers->indices.push_back(i);
			}
			else if (comparator(delta_y, 0) && abs(delta_z) / abs(delta_y) < 10) {
				inliers->indices.push_back(i);
			}
		}
	}
	// 删除这些点
	// TODO: 分界线为竖着的时候，分割不掉
	ExtractIndices<PT> extract;
	extract.setInputCloud(source_cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*source_cloud);
	myVisualization2<PT>(source_cloud, min_cloud, "removed");
}

template<typename PT>
void find_and_remove_min_z_points(typename PointCloud<PT>::Ptr source_cloud, int direction) {
	// 划分区间，对点云进行切片
	PT min{}, max{};
	getMinMax3D(*source_cloud, min, max);
	float cloud_mean = (min.y + max.y) / 2;
	float gap = 10.0;
	int n = (max.x - min.x) / gap + 1;
	std::vector<double> intervals(n);
	for (int i = 0; i < n; ++i) {
		intervals[i] = min.x + gap * i;
	}

	//myVisualization<PT>(source_cloud, "half");
	typename PointCloud<PT>::Ptr min_cloud(new PointCloud<PT>);

	// 遍历点云，找到最小Z值的点
	std::vector<PT> interval_min_z(n, { 0, 0, std::numeric_limits<float>::max() });
	std::vector<PT> interval_min_y(n, { 0, std::numeric_limits<float>::max(), 0 });
	std::vector<PT> interval_max_y(n, { 0, std::numeric_limits<float>::min(), 0 });

	//#pragma omp parallel for 
	// 遍历点云
	// 提取XY平面上的点
	std::vector<std::vector<Eigen::Vector2d>> interval_points(n);
	for (int i = 0; i < source_cloud->points.size(); i++) {
		// 判断在哪个区间
		int n = (source_cloud->points[i].x - min.x) / gap;
		interval_points[n].push_back(Eigen::Vector2d(source_cloud->points[i].y, source_cloud->points[i].z));
		// 更新最小Z值点
		if (source_cloud->points[i].z < interval_min_z[n].z) {
			interval_min_z[n] = source_cloud->points[i];
		}
		if (source_cloud->points[i].y < interval_min_y[n].y) {
			interval_min_y[n] = source_cloud->points[i];
		}
		if (source_cloud->points[i].y > interval_max_y[n].y) {
			interval_max_y[n] = source_cloud->points[i];
		}
	}

	// direction: 0 -> 左，删除较大的，1 -> 右，删除较小的
	std::function<bool(int, int)> comparator;
	std::function<int(int, int)> operation;
	int thre_tan;
	std::vector<PT> thre_interval_y;
	if (direction == 0) {
		comparator = [](int a, int b) { return a >= b; }; // 大于
		operation = [](int a, int b) { return a - b; };  // 减法
		thre_tan = 1;
		thre_interval_y = interval_min_y;
	}
	else
	{
		comparator = [](int a, int b) { return a <= b; };  // 小于
		operation = [](int a, int b) { return a + b; };  // 加法
		thre_tan = -1;
		thre_interval_y = interval_max_y;
	}
	std::vector<int> valid_interval_index;
	for (int i = 0; i < interval_min_z.size(); i++) {
		float width = interval_max_y[i].y - interval_min_y[i].y;
		//if (width > (max.y - min.y) * 0.6) {
			//if (comparator(operation(interval_min_z[i].y, 0.1 * width), thre_interval_y[i].y)) {
		if (std::abs(interval_min_z[i].z - thre_interval_y[i].z) > 50) {
			min_cloud->push_back(interval_min_z[i]);
			interval_min_z[i].x = 99999;
			valid_interval_index.push_back(i);
		}
		//}
	}
	//myVisualization2<PT>(source_cloud, min_cloud, "min cloud");

	// 利用极小值点判断是否有V
	int min_extrema_count = 0;
	int total_count = 0;
	for (int i = 0; i < valid_interval_index.size(); i = i + 5) {
		total_count++;
		int index = valid_interval_index[i];
		std::vector<Eigen::Vector2d> points = interval_points[index];
		// 拟合多项式
		int degree = 3; // 设置多项式的阶数
		Eigen::VectorXd coefficients;
		fitPolynomial(points, degree, coefficients);
		// 计算多项式的导数
		Eigen::VectorXd derivative = polynomialDerivative(coefficients);
		// 极值点的坐标
		std::vector<double> extrema = findExtrema(derivative, interval_min_y[index].y, interval_max_y[index].y, direction);

		if (extrema.size() == 2) {
			min_extrema_count += 1;
		}
		else if (extrema.size() == 1) {
			double second_derivative = evaluateSecondDerivative(coefficients, extrema[0]);
			if (second_derivative > 0) {
				min_extrema_count += 1;
			}
		}
	}
	//cout << count / valid_interval_index.size();
	if (min_extrema_count * 1.0 / total_count * 1.0 < 0.5) {
		//cout << min_extrema_count / total_count;
		return;
	}
	//myVisualization2<PT>(source_cloud, min_cloud, "min cloud");

	PointIndices::Ptr inliers(new PointIndices());
	// 遍历点云
	for (int i = 0; i < source_cloud->points.size(); ++i) {
		// 判断在哪个区间
		int n = (source_cloud->points[i].x - min.x) / gap;
		float delta_y = source_cloud->points[i].y - interval_min_z[n].y;
		float delta_z = source_cloud->points[i].z - interval_min_z[n].z;
		if (interval_min_z[n].x == 99999) {
			if (comparator(source_cloud->points[i].y, operation(interval_min_z[n].y, 5)) && (source_cloud->points[i].z < (interval_min_z[n].z + 50))) {
				inliers->indices.push_back(i);
			}
			else if (comparator(delta_y, 0) && abs(delta_z) / abs(delta_y) < 10) {
				inliers->indices.push_back(i);
			}
		}
	}
	// 删除这些点
	// TODO: 分界线为竖着的时候，分割不掉
	ExtractIndices<PT> extract;
	extract.setInputCloud(source_cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*source_cloud);
	//myVisualization2<PT>(source_cloud, min_cloud, "removed");
}

template<typename PT>
void removePeakV(typename PointCloud<PT>::Ptr source_cloud) {
	/* 计算Normal change rate(From CC)*/
	std::vector<double> ncr = getNormalChangeRate<PT>(source_cloud, 30);
	PointIndices::Ptr inliers(new PointIndices());
	for (int i = 0; i < source_cloud->points.size(); i++) {
		if (ncr[i] >= 0.15) {
			inliers->indices.push_back(i);
		}
	}

	// 遍历点云，找到最小Z值的点
	PT min{}, max{};
	getMinMax3D(*source_cloud, min, max);
	int n = (max.x - min.x) / 10.0;
	PT min_point{};
	int min_idx = -1;
	typename PointCloud<PT>::Ptr min_cloud(new PointCloud<PT>);
	std::vector<double> intervals(n);
	for (int i = 0; i < n - 1; ++i) {
		intervals[i] = min.x + (max.x - min.x) * i / (n - 1);
	}
	//#pragma omp parallel for
	for (int i = 0; i < intervals.size() - 1; i++) {
		float minZ = std::numeric_limits<float>::max();  // 初始化为最大浮点数值
		float x1 = intervals[i];
		float x2 = intervals[i + 1];
		// 遍历点云
		for (int i = 0; i < source_cloud->points.size(); ++i) {
			// 检查点的X坐标是否在指定范围内
			if (source_cloud->points[i].x >= x1 && source_cloud->points[i].x <= x2) {
				// 更新最小Z值点
				if (source_cloud->points[i].z < minZ) {
					minZ = source_cloud->points[i].z;
					min_point = source_cloud->points[i];
					min_idx = i;
				}
			}
		}
		//#pragma omp critical
		{
			// 存储最小Z值点及其对应的X坐标范围
			min_cloud->push_back(min_point);
			inliers->indices.push_back(min_idx);
		}
	}
	//myVisualization2<PT>(source_cloud, min_cloud, "min_cloud");

	typename PointCloud<PT>::Ptr max_ncr_cloud(new PointCloud<PT>);
	ExtractIndices<PointXYZ> extract;
	extract.setInputCloud(source_cloud);
	extract.setIndices(inliers);
	extract.setNegative(false); // true删除，false保留指定点
	extract.filter(*max_ncr_cloud);
	//myVisualization2<PT>(source_cloud, max_ncr_cloud, "bigger ncr");
	// 将点云按z值排序
	//std::vector<int> indices(min_cloud->points.size());
	//std::iota(indices.begin(), indices.end(), 0);
	//std::sort(indices.begin(), indices.end(), [&min_cloud](int a, int b) {
	//	return min_cloud->points[a].z < min_cloud->points[b].z;
	//	});

	// 计算前80%的点数
	//int num_points_to_keep = static_cast<int>(min_cloud->points.size() * 0.9);
	// 创建新的点云对象，包含筛选后的点
	//typename PointCloud<PT>::Ptr filtered_cloud(new PointCloud<PT>);
	//filtered_cloud->width = num_points_to_keep;
	//filtered_cloud->height = 1;
	//filtered_cloud->is_dense = true;
	//filtered_cloud->points.resize(num_points_to_keep);
	//for (int i = 0; i < num_points_to_keep; ++i) {
	//	filtered_cloud->points[i] = min_cloud->points[indices[i]];
	//}

	//半径滤波
	RadiusOutlierRemoval<PT> radiusoutlier;  //创建半径滤波器
	radiusoutlier.setInputCloud(max_ncr_cloud); //输入点云
	radiusoutlier.setRadiusSearch(40);   //搜索半径
	radiusoutlier.setMinNeighborsInRadius(10);//邻域点阈值
	radiusoutlier.filter(*max_ncr_cloud);
	//myVisualization<PT>(max_ncr_cloud, "guo lv");


	// 初始化KD树
	KdTreeFLANN<PT> kdtree;
	kdtree.setInputCloud(source_cloud);
	// 定义搜索半径
	double search_radius = 30; // 根据实际需要调整
	// 获取min_cloud在source_cloud中的近邻点
	//PointIndices::Ptr inliers(new PointIndices());
#pragma omp parallel
	{
		std::vector<int> point_idx_radius_search;
		std::vector<float> point_radius_squared_distance;
#pragma omp for
		for (int i = 0; i < max_ncr_cloud->points.size(); ++i)
		{
			point_idx_radius_search.clear();
			point_radius_squared_distance.clear();
			if (kdtree.radiusSearch(max_ncr_cloud->points[i], search_radius, point_idx_radius_search, point_radius_squared_distance) > 0)
			{
#pragma omp critical
				{
					inliers->indices.insert(inliers->indices.end(), point_idx_radius_search.begin(), point_idx_radius_search.end());
				}
			}
		}
	}

	// 去重
	std::sort(inliers->indices.begin(), inliers->indices.end());
	inliers->indices.erase(std::unique(inliers->indices.begin(), inliers->indices.end()), inliers->indices.end());

	typename PointCloud<PT>::Ptr source_cloud_filtered(new PointCloud<PT>);
	// 删除
	//ExtractIndices<PointXYZ> extract;
	extract.setInputCloud(source_cloud);
	extract.setIndices(inliers);
	extract.setNegative(false); // true删除，false保留指定点
	extract.filter(*source_cloud_filtered);
	//myVisualization2<PT>(source_cloud, source_cloud_filtered, "min_cloud");

	extract.setInputCloud(source_cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*source_cloud);
}

template<typename PT>
void removePeakV2(typename PointCloud<PT>::Ptr source_cloud) {
	// 分割点云为两部分（根据Y值）
	PT min{}, max{};
	getMinMax3D(*source_cloud, min, max);
	int mid_line = (max.y + min.y) / 2;
	typename PointCloud<PT>::Ptr left_half(new PointCloud<PT>);
	typename PointCloud<PT>::Ptr right_half(new PointCloud<PT>);
	for (const auto& point : *source_cloud) {
		if (point.y > mid_line) {
			left_half->points.push_back(point);
		}
		else {
			right_half->points.push_back(point);
		}
	}
	getMinMax3D(*left_half, min, max);
	if (max.z - min.z > 200)find_and_remove_min_z_points<PT>(left_half, 0);

	getMinMax3D(*right_half, min, max);
	if (max.z - min.z > 200)find_and_remove_min_z_points<PT>(right_half, 1);

	*source_cloud = *left_half + *right_half;
	//myVisualization<PT>(source_cloud, "removed V");
}

template<typename PT>
void getMaxCluster(typename PointCloud<PT>::Ptr source_cloud, int process) {
	if (process) {
		// 通过下采样的点云进行快速分割，然后将分割结果的包围盒应用到原始点云
		typename PointCloud<PT>::Ptr down_cloud(new PointCloud<PT>);
		*down_cloud = randomSampleForce<PT>(source_cloud, 0.5);

		typename search::KdTree<PT>::Ptr tree(new search::KdTree<PT>);
		tree->setInputCloud(down_cloud);
		int num_points = down_cloud->points.size();

		// 欧式聚类
		//EuclideanClusterExtraction<PT> ec;
		DBSCANKdtreeCluster<PT> ec;
		ec.setCorePointMinPts(15); // 判定为核心点需要的最小点数
		ec.setClusterTolerance(30); // 设置聚类的容差
		ec.setMinClusterSize(0);   // 设置最小的聚类大小
		ec.setMaxClusterSize(num_points);  // 设置最大的聚类大小
		ec.setSearchMethod(tree);
		ec.setInputCloud(down_cloud);
		std::vector<PointIndices> cluster_indices;
		ec.extract(cluster_indices);


		// 获取最大的聚类
		typename PointCloud<PT>::Ptr maxCluster(new PointCloud<PT>);
		std::sort(cluster_indices.begin(), cluster_indices.end(), [](PointIndices a, PointIndices b) {
			return a.indices.size() > b.indices.size();
			});
		copyPointCloud(*down_cloud, cluster_indices[0], *maxCluster);
		//myVisualization2<PT>(down_cloud, maxCluster, "down sample");

		PT min{}, max{};
		getMinMax3D(*maxCluster, min, max);
		float detla = 10;
		PassThrough<PT> pass;
		pass.setFilterFieldName("x");
		pass.setFilterLimits(min.x - detla, max.x + detla);
		pass.setInputCloud(source_cloud);
		pass.filter(*source_cloud);

		pass.setFilterFieldName("y");
		pass.setFilterLimits(min.y - detla, max.y + detla);
		pass.setInputCloud(source_cloud);
		pass.filter(*source_cloud);

		pass.setFilterFieldName("z");
		pass.setFilterLimits(min.z - detla, max.z + detla);
		pass.setInputCloud(source_cloud);
		pass.filter(*source_cloud);
	}
	else {
		typename search::KdTree<PT>::Ptr tree(new search::KdTree<PT>);
		tree->setInputCloud(source_cloud);
		int num_points = source_cloud->points.size();

		DBSCANKdtreeCluster<PT> ec;
		//EuclideanClusterExtraction<PT> ec;
		ec.setCorePointMinPts(20); // 设置核心点最小点数
		ec.setClusterTolerance(25); // 设置聚类的容差
		ec.setMinClusterSize(0);   // 设置最小的聚类大小
		ec.setMaxClusterSize(num_points);  // 设置最大的聚类大小
		ec.setSearchMethod(tree);
		ec.setInputCloud(source_cloud);
		std::vector<PointIndices> cluster_indices;
		ec.extract(cluster_indices);

		// 获取最大的聚类
		typename PointCloud<PT>::Ptr maxCluster(new PointCloud<PT>);
		std::sort(cluster_indices.begin(), cluster_indices.end(), [](PointIndices a, PointIndices b) {
			return a.indices.size() > b.indices.size();
			});
		copyPointCloud(*source_cloud, cluster_indices[0], *maxCluster);
		*source_cloud = *maxCluster;
	}
	//return cluster_indices.size();
}

template<typename PT>
void removeNoise(typename PointCloud<PT>::Ptr source_cloud) {
	StatisticalOutlierRemoval<PT> sor;
	sor.setInputCloud(source_cloud);
	sor.setMeanK(30);
	sor.setStddevMulThresh(3.0);
	//sor.setNegative(true);
	sor.filter(*source_cloud);
}

template<typename PT>
void removeOtherObj(typename PointCloud<PT>::Ptr source_cloud, int process) {
	if (process) {
		//cout << "first seg" << "-";
		getMaxCluster<PT>(source_cloud, 1);
		//myVisualization<PT>(source_cloud, "first seg");
		PT min{}, max{};
		getMinMax3D(*source_cloud, min, max);

		if (max.y - min.y > 600) {
			//cout << "remove V" << "-";
			removePeakV2<PT>(source_cloud);
			//cout << "second seg" << "-";
			getMaxCluster<PT>(source_cloud, 0);
		}
	}
	else {
		getMaxCluster<PT>(source_cloud, 0);
	}
}

template<typename PT>
void smoothByMLS(typename PointCloud<PT>::Ptr source_cloud, typename PointCloud<PointNormal>::Ptr cloud_normal) {
	typename search::KdTree<PT>::Ptr kdtree;
	MovingLeastSquares<PT, PointNormal> mls;
	mls.setInputCloud(source_cloud);
	mls.setComputeNormals(true);
	mls.setSearchRadius(30); // 拟合半径
	mls.setPolynomialOrder(true); // 利用多项式
	mls.setPolynomialOrder(2); // 三阶
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

template<typename PT>
void saveScreenshot(typename PointCloud<PT>::Ptr source_cloud, std::string file_path) {
	// 创建PCL可视化对象
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D Viewer", false)); // 第二个参数设为 false 禁用交互窗口
	viewer->setBackgroundColor(0, 0, 0);
	visualization::PointCloudColorHandlerCustom<PT> cloud_color(source_cloud, 255, 255, 255);
	viewer->addPointCloud<PT>(source_cloud, cloud_color, "source_cloud");
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud");

	// 设置俯视图角度
	viewer->setCameraPosition(0, -1, 0, 0, 0, 1, 0, 0, 1);

	// 强制渲染以更新内部状态
	viewer->getRenderWindow()->OffScreenRenderingOn(); // 启用离屏渲染
	viewer->getRenderWindow()->Render();

	// 保存截图
	viewer->getRenderWindow()->Render();
	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	windowToImageFilter->SetInput(viewer->getRenderWindow());
	//windowToImageFilter->SetMagnification(1); // 放大比例
	windowToImageFilter->SetInputBufferTypeToRGB();
	windowToImageFilter->ReadFrontBufferOff();
	windowToImageFilter->Update();

	vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
	writer->SetFileName(file_path.c_str());
	writer->SetInputConnection(windowToImageFilter->GetOutputPort());
	writer->Write();
}