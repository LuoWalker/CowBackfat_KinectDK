#pragma once
#include "GetObj.hpp"
#include "utils.h"
#include <boost/container_hash/is_contiguous_range.hpp>
#include <cmath>
#include <cstdlib>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/MatrixBase.h>
#include <Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Transform.h>
#include <limits>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h> // 拟合直线
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <vector>


using namespace pcl;

struct IMUSAMPLE
{
	int cur_frame;
	float acc_x;
	float acc_y;
	float acc_z;
};

// 计算曲率
template<typename PT>
double calculateCurvature(typename PointCloud<PT>::Ptr cloud, const std::vector<int>& indices) {
	Eigen::MatrixXf points_matrix(indices.size(), 3);
	for (size_t i = 0; i < indices.size(); ++i) {
		points_matrix(i, 0) = cloud->points[indices[i]].x;
		points_matrix(i, 1) = cloud->points[indices[i]].y;
		points_matrix(i, 2) = cloud->points[indices[i]].z;
	}
	Eigen::Vector3f mean = points_matrix.colwise().mean();
	Eigen::MatrixXf centered = points_matrix.rowwise() - mean.transpose();
	Eigen::MatrixXf cov = (centered.adjoint() * centered) / double(points_matrix.rows() - 1);
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);
	return eig.eigenvalues()(0);
}

template<typename PT>
void rotationByVecotr(typename PointCloud<PT>::Ptr source_cloud, Eigen::Vector3d v, int des_axis) {
	// desAxis: 0-x轴，1-y轴，2-z轴
	// 将目标向量转化为单位向量
	v.normalize();
	Eigen::Vector3d axis;
	double angle;
	switch (des_axis)
	{
	case 0:
		// 由于我们要将des_axis轴旋转到向量v的位置，因此我们需要找到从v到des_axis轴的旋转轴
		// 这可以通过计算两个向量的叉积来得到，叉积将垂直于旋转平面
		axis = v.cross(Eigen::Vector3d::UnitX());
		axis.normalize();
		// 计算旋转角度，这里我们使用acos来计算夹角，并将其转换为弧度
		angle = acos(v.dot(Eigen::Vector3d::UnitX()));
		break;
	case 1:
		axis = v.cross(Eigen::Vector3d::UnitY());
		axis.normalize();
		angle = acos(v.dot(Eigen::Vector3d::UnitY()));
		break;
	case 2:
		axis = v.cross(Eigen::Vector3d::UnitZ());
		axis.normalize();
		angle = acos(v.dot(Eigen::Vector3d::UnitZ()));
		break;
	}

	// 创建仿射矩阵
	Eigen::Affine3d transform = Eigen::Affine3d::Identity();
	PT min{}, max{};
	getMinMax3D(*source_cloud, min, max);
	//transform.translation() << -min.x, -min.y, -min.z; // 平移
	transform.translation() << 0, 0, 0; // 平移
	transform.rotate(Eigen::AngleAxisd(angle, axis)); // 旋转

	transformPointCloud(*source_cloud, *source_cloud, transform);
}

template<typename PT>
void rotationByVecotr(typename PointCloud<PT>::Ptr source_cloud, typename PointCloud<PT>::Ptr cloud_line, Eigen::Vector3d v, int des_axis) {
	// desAxis: 0-x轴，1-y轴，2-z轴
	// 将目标向量转化为单位向量
	v.normalize();
	Eigen::Vector3d axis;
	double angle;
	switch (des_axis)
	{
	case 0:
		// 由于我们要将des_axis轴旋转到向量v的位置，因此我们需要找到从v到des_axis轴的旋转轴
		// 这可以通过计算两个向量的叉积来得到，叉积将垂直于旋转平面
		axis = v.cross(Eigen::Vector3d::UnitX());
		axis.normalize();
		// 计算旋转角度，这里我们使用acos来计算夹角，并将其转换为弧度
		angle = acos(v.dot(Eigen::Vector3d::UnitX()));
		break;
	case 1:
		axis = v.cross(Eigen::Vector3d::UnitY());
		axis.normalize();
		angle = acos(v.dot(Eigen::Vector3d::UnitY()));
		break;
	case 2:
		axis = v.cross(Eigen::Vector3d::UnitZ());
		axis.normalize();
		angle = acos(v.dot(Eigen::Vector3d::UnitZ()));
		break;
	}

	// 创建仿射矩阵
	Eigen::Affine3d transform = Eigen::Affine3d::Identity();
	transform.translation() << 0, 0, 0; // 平移
	transform.rotate(Eigen::AngleAxisd(angle, axis)); // 旋转

	transformPointCloud(*source_cloud, *source_cloud, transform);
	transformPointCloud(*cloud_line, *cloud_line, transform);

	transform = Eigen::Affine3d::Identity();
	PT min{}, max{};
	getMinMax3D(*cloud_line, min, max);
	transform.translation() << -min.x, -min.y, -min.z; // 平移
	transformPointCloud(*source_cloud, *source_cloud, transform);
	transformPointCloud(*cloud_line, *cloud_line, transform);
}

template<typename PT>
void limitZ(typename PointCloud<PT>::Ptr source_cloud) {
	PT min{}, max{};

	PassThrough<PT> pass;
	getMinMax3D(*source_cloud, min, max);

	pass.setInputCloud(source_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(max.z - 400, max.z);
	pass.filter(*source_cloud);
}

static float vectorAngle(const Eigen::Vector3f& x, const Eigen::Vector3f& y) {
	float Lx = x.norm();
	float Ly = y.norm();
	float cosAngle = x.dot(y) / (Lx * Ly);
	float angle = std::acos(cosAngle);
	return angle * 180 / M_PI;
}

template<typename PT>
void downSampleNormal(typename PointCloud<PT>::Ptr source_cloud) {
	// 计算法向量和曲率
	typename search::KdTree<PT>::Ptr kdtree(new search::KdTree<PT>);
	kdtree->setInputCloud(source_cloud);

	NormalEstimationOMP<PT, Normal> ne;
	ne.setInputCloud(source_cloud);
	ne.setSearchMethod(kdtree);
	ne.setKSearch(100);
	PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>);
	ne.compute(*cloud_normals);

	// 设置阈值
	double K1 = 0.5; // 夹角阈值（可以根据实际情况调整）
	double K2 = 0.1; // 曲率均方差阈值（可以根据实际情况调整）

	// 保留点云、待精简和待删除点集
	typename PointCloud<PT>::Ptr cloud_keep(new PointCloud<PT>);
	typename PointCloud<PT>::Ptr cloud_filter(new PointCloud<PT>);
	typename PointCloud<PT>::Ptr cloud_delete(new PointCloud<PT>);

	std::vector<int> idx;
	std::vector<float> dis;
	Eigen::Vector3f current_normal;
	std::vector<Eigen::Vector3f> knn_normal;
	float normal_angle = 0.0;

	// 遍历所有点
//#pragma omp parallel for 
	for (int i = 0; i < source_cloud->size(); ++i) {
		if (i % 1000 == 0)
			cout << i << " ";
		// 获取当前点法向量
		current_normal = cloud_normals->points[i].getNormalVector3fMap();
		// 获取当前点邻域内的法向量
		kdtree->nearestKSearch(source_cloud->points[i], 21, idx, dis);
		for (int j = 1; j < idx.size(); ++j) {
			knn_normal.push_back(cloud_normals->points[idx[j]].getNormalVector3fMap());
		}
		// 计算当前点法向量与邻域内法向量均值的夹角
		normal_angle = calculateAngleA(current_normal, knn_normal);
		// 判断法向量夹角A是否小于阈值K1
		if (normal_angle < K1) {
			cloud_keep->push_back(source_cloud->points[i]);
			continue; // 保留点
		}
		else {
			// 计算邻域点集曲率
			std::vector<double> curvatures;
			for (const auto& idx : idx) {
				curvatures.push_back(calculateCurvature(source_cloud, { idx }));
			}

			// 计算曲率均值
			double mean_curvature = std::accumulate(curvatures.begin(), curvatures.end(), 0.0) / curvatures.size();

			// 计算曲率均方差σ_D
			double sigma_D = calculateCurvatureStdDev(curvatures, mean_curvature);

			// 判断曲率均方差σ_D是否大于阈值K2
			if (sigma_D > K2) {
				//#pragma omp critical
				cloud_filter->push_back(source_cloud->points[i]); // 加入待精简点集
			}
			else {
				//#pragma omp critical
				cloud_delete->push_back(source_cloud->points[i]); // 加入待删除点集
			}
		}
	}

	UniformSampling<PT> uniform_sampling;
	uniform_sampling.setInputCloud(cloud_filter);
	uniform_sampling.setRadiusSearch(5);
	uniform_sampling.filter(*cloud_filter);

	uniform_sampling.setInputCloud(cloud_delete);
	uniform_sampling.setRadiusSearch(9);
	uniform_sampling.filter(*cloud_delete);

	typename PointCloud<PT>::Ptr cloud_final(new PointCloud<PT>);
	*source_cloud = *cloud_keep + *cloud_filter + *cloud_delete;
	//myVisualization2<PT>(cloud_filter, cloud_delete, "down sample two part");
}

template<typename PT>
PointCloud<PT> normalDownSample(typename PointCloud<PT>::Ptr source_cloud) {
	typename PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
	// 计算法向量
	NormalEstimationOMP<PT, Normal> ne;
	ne.setInputCloud(source_cloud);
	typename search::KdTree<PT>::Ptr tree(new search::KdTree<PT>());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(30); // 设置搜索半径
	ne.compute(*normals);

	// 使用法向量空间采样进行下采样
	PointCloud<PT> sampled_cloud;
	NormalSpaceSampling<PT, Normal> nss;
	nss.setInputCloud(source_cloud);
	nss.setNormals(normals);
	nss.setBins(4, 4, 4); // 设置法向量空间的bin数
	nss.setSample(3000); // 设置下采样点的数量
	nss.filter(sampled_cloud);
	return sampled_cloud;
}

template<typename PT>
PointCloud<PT> normalization(typename PointCloud<PT>::Ptr source_cloud) {

	// 计算质心
	Eigen::Vector4f centroid;
	compute3DCentroid(*source_cloud, centroid);

	// 将点云减去质心
	PointCloud<PT> normalized_cloud;
	demeanPointCloud(*source_cloud, centroid, normalized_cloud);
	return normalized_cloud;
}

template<typename PT>
typename PointCloud<PT>::Ptr normalizationZ(typename PointCloud<PT>::Ptr source_cloud) {
	// 创建拟合对象
	SACSegmentation<PT> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(SACMODEL_PLANE);
	seg.setMethodType(SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(170);

	seg.setInputCloud(source_cloud);

	// 计算索引和平面系数
	PointIndices::Ptr inliers(new PointIndices());
	ModelCoefficients::Ptr coefficients(new ModelCoefficients());
	seg.segment(*inliers, *coefficients);

	// 提取拟合平面点
	ExtractIndices<PT> extract;
	extract.setInputCloud(source_cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	typename PointCloud<PT>::Ptr back_plane(new PointCloud<PT>);
	extract.filter(*back_plane);

	// 计算法线向量与Z轴的夹角，并得到旋转矩阵
	Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
	float angle = acos(normal.dot(Eigen::Vector3f::UnitZ()));
	// 构建旋转的变换矩阵
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate(Eigen::AngleAxisf(angle, normal.cross(Eigen::Vector3f::UnitZ()).normalized()));

	// 将拟合得到的平面旋转到与XOY平面平行
	typename PointCloud<PT>::Ptr rotated_plane(new PointCloud<PT>);
	transformPointCloud(*back_plane, *rotated_plane, transform);
	return rotated_plane;
}

template<typename PT>
void normalizationZ(typename PointCloud<PT>::Ptr source_cloud, IMUSAMPLE imu_sample) {
	//// 遍历点云并取反Z值
	//for (auto& point : *source_cloud) {
	//	point.z = -point.z;
	//}
	// imu传感器坐标系 xyz
	// Eigen::Vector3d v = { imu_samples[1],imu_samples[2],imu_samples[3] };
	// 相机坐标系
	Eigen::Vector3d v = { imu_sample.acc_y,-1.0 * imu_sample.acc_z,imu_sample.acc_x };
	// 将Z轴与重力加速度向量对齐
	rotationByVecotr<PT>(source_cloud, v, 2);

	// 遍历点云并取反Z值
	for (auto& point : *source_cloud) {
		point.z = -point.z;
	}
	limitZ<PT>(source_cloud);
}

template<typename PT>
void normalizationXOY(typename PointCloud<PT>::Ptr source_cloud, typename PointCloud<PT>::Ptr cloud_line) {
	// 遍历点云，找到最大Z值的点
	PT min{}, max{};
	getMinMax3D(*source_cloud, min, max);
	int n = (max.x - min.x) / 10.0;
	PT max_point;
	typename PointCloud<PT>::Ptr maxz_cloud(new PointCloud<PT>);


	std::vector<double> intervals(n);
	for (int i = 0; i < n; ++i) {
		intervals[i] = min.x + (max.x * 0.7 - min.x) * i / (n - 1);
	}
	float maxZ = std::numeric_limits<float>::lowest();  // 初始化为最小浮点数值
	for (int i = 0; i < intervals.size() - 1; i++) {
		//float minZ = std::numeric_limits<float>::max();  // 初始化为最大浮点数值
		float x1 = intervals[i];
		float x2 = intervals[i + 1];

		// 遍历点云
		for (const auto& point : source_cloud->points) {
			// 检查点的X坐标是否在指定范围内
			if (point.x >= x1 && point.x <= x2) {
				// 更新最大Z值点
				if (point.z > maxZ) {
					maxZ = point.z;
					max_point = point;
				}
			}
		}
		// 存储最大Z值点及其对应的X坐标范围
		maxz_cloud->push_back(max_point);
		maxZ = std::numeric_limits<float>::lowest();
	}

	//-----------------------------拟合直线-----------------------------
	typename SampleConsensusModelLine<PT>::Ptr model_line(new SampleConsensusModelLine<PT>(maxz_cloud));
	RandomSampleConsensus<PT> ransac(model_line);
	ransac.setDistanceThreshold(10);	//内点到模型的最大距离
	ransac.setMaxIterations(1000);		//最大迭代次数
	ransac.computeModel();				//直线拟合
	//--------------------------根据索引提取内点------------------------
	std::vector<int> inliers;
	ransac.getInliers(inliers);
	copyPointCloud<PT>(*maxz_cloud, inliers, *cloud_line);

	//----------------------------输出模型参数--------------------------
	Eigen::VectorXf coef;
	ransac.getModelCoefficients(coef);
	Eigen::Vector3d v = { coef[3],coef[4],coef[5] };
	if (coef[3] < 0) {
		v = { -coef[3],-coef[4],-coef[5] };
	}

	//myVisualization2<PT>(source_cloud, cloud_line, "line");
	rotationByVecotr<PT>(source_cloud, cloud_line, v, 0);
	//rotationByVecotr<PT>(cloud_line, v, 0);
}

template<typename PT>
void preProcess(typename PointCloud<PT>::Ptr source_cloud) {
	// 遍历点云并取反Z值
	for (auto& point : *source_cloud) {
		point.z = -point.z;
	}

	PT min{}, max{};

	PassThrough<PT> pass;
	getMinMax3D(*source_cloud, min, max);
	pass.setInputCloud(source_cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(min.x, min.x + 800);
	pass.filter(*source_cloud);
	//myVisualization<PT>(source_cloud, "limit x");


	//getMinMax3D(*source_cloud, min, max);
	//pass.setInputCloud(source_cloud);
	//pass.setFilterFieldName("z");
	//pass.setFilterLimits(max.z - 400, max.z);
	//pass.filter(*source_cloud);
	//myVisualization<PT>(source_cloud, "limit z");

}

template<typename PT>
void limitArea(typename PointCloud<PT>::Ptr source_cloud) {
	PT min{}, max{};
	//cout << "X轴最值" << min.x << ' ' << max.x << endl;
	//cout << "Y轴最值" << min.y << ' ' << max.y << endl;
	//cout << "Z轴最值" << min.z << ' ' << max.z << endl;

	PassThrough<PT> pass;
	getMinMax3D(*source_cloud, min, max);

	pass.setInputCloud(source_cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(min.x, min.x + 700);
	pass.filter(*source_cloud);

	//pass.setFilterFieldName("y");
	//pass.setFilterLimits(min.y, min.y + 750);
	//pass.filter(*result_cloud);

	pass.setInputCloud(source_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(max.z - 300, max.z);
	//pass.setFilterLimits(min.z, min.z + 500);
	pass.filter(*source_cloud);

	//getMinMax3D(*source_cloud, min, max);
	//Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	//transform.translation() << -min.x, -min.y, -min.z;
	//transform.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));
	//transformPointCloud(*source_cloud, *source_cloud, transform);
}

template<typename PT>
bool keyPointPosition(typename PointCloud<PT>::Ptr source_cloud) {
	/*
	* 	遍历点云，找到Y值跨度最大的区间，为腰角
	*   得到区间右端X值，对点云进行裁切
	*/

	// 预处理
	PassThrough<PT> pass;
	PT min{}, max{};
	getMinMax3D(*source_cloud, min, max);
	//pass.setInputCloud(source_cloud);
	//pass.setFilterFieldName("z");
	//pass.setFilterLimits(max.z - 300, max.z);
	//pass.filter(*source_cloud);

	// 划分区间
	int max_width = 0;
	float gap = 10.0;
	int n = (max.x - min.x) / gap + 1;
	std::vector<double> intervals(n);
	for (int i = 0; i < n; ++i) {
		intervals[i] = min.x + i * gap;
	}

	// 统计每个区间的跨度
	PointCloud<PT> bin_cloud;
	std::vector<int> width(n, 0);
	std::vector<PT> interval_min_y(n, { 0, std::numeric_limits<float>::max(), 0 });
	std::vector<PT> interval_max_y(n, { 0, std::numeric_limits<float>::min(), 0 });
	for (int i = 0; i < source_cloud->points.size(); i++) {
		// 判断在哪个区间
		int n = (source_cloud->points[i].x - min.x) / gap;
		// 更新最值点
		if (source_cloud->points[i].y < interval_min_y[n].y) {
			interval_min_y[n] = source_cloud->points[i];
		}
		if (source_cloud->points[i].y > interval_max_y[n].y) {
			interval_max_y[n] = source_cloud->points[i];
		}
	}
	int tail_index = 0;
	for (int i = 0; i < interval_min_y.size() / 3; i++) {
		width[i] = interval_max_y[i].y - interval_min_y[i].y;
		if (width[i] > 50 && tail_index == 0) {
			tail_index = i;
		}
		else if (width[i] < 50 && tail_index != 0) {
			tail_index = 0;
		}
	}

	// 找到跨度最大的区间
	int max_index = std::max_element(width.begin(), width.end()) - width.begin();
	if (max_index < n / 2) { max_index = n - 2; }

	pass.setInputCloud(source_cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(intervals[tail_index + 1], intervals[max_index + 1] + 100);
	pass.filter(*source_cloud);
	//myVisualization<PT>(source_cloud, "cut");
	return true;
}

template<typename PT>
int filterByVolume(typename PointCloud<PT>::Ptr source_cloud) {
	/*
		0:舍弃
		1:保留，无需镜像，直接通过
		2:保留，需要镜像，返回镜像后的点云
	*/
	PointCloud<PT> left_cloud, right_cloud, middle_cloud;
	PT min{}, max{};

	// 如果整个点云都很短，不包括POI，直接丢弃
	getMinMax3D(*source_cloud, min, max);
	if ((max.x - min.x) < 500) {
		return 0;
	}

	// 已经将脊柱移到x轴正方向，所以这里只需要判断y轴，将其分为左中右三部分判断
	PassThrough<PT> pass;
	pass.setInputCloud(source_cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(min.y, 0);
	pass.filter(right_cloud);

	pass.setInputCloud(source_cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(0, max.y);
	pass.filter(left_cloud);

	pass.setInputCloud(source_cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-50, 50);
	pass.filter(middle_cloud);

	// 判断脊柱长度，不然没有办法进行镜像操作，最起码要有500mm
	getMinMax3D(middle_cloud, min, max);
	if ((max.x - min.x) < 500) { return 0; }


	// 排除脊柱定位异常的情况，比如脊柱不完整，在通过最大Z值定位脊柱时会定位到点云边界
	// 此时会出现只有轴的一侧点多，另一侧点极少，所以需要排除这种情况
	getMinMax3D(right_cloud, min, max);
	if ((max.x - min.x) < 200 || (max.y - min.y) < 200) { return 0; }

	getMinMax3D(left_cloud, min, max);
	if ((max.x - min.x) < 200 || (max.y - min.y) < 200) { return 0; }

	// 少的一端用于判断是否需要镜像，多的一端用于镜像
	PointCloud<PT> less_cloud, more_cloud;
	less_cloud = left_cloud.points.size() > right_cloud.points.size() ? right_cloud : left_cloud;
	less_cloud = less_cloud + middle_cloud;
	more_cloud = left_cloud.points.size() > right_cloud.points.size() ? left_cloud : right_cloud;
	more_cloud = right_cloud + middle_cloud;
	if (less_cloud.points.size() < 0.5 * more_cloud.points.size()) {
		//	// 生成镜像点云
		//	typename PointCloud<PT>::Ptr cloud_mirror(new PointCloud<PT>());
		//	*cloud_mirror = more_cloud;
		//	for (auto& point : cloud_mirror->points) {
		//		point.y = -point.y; // 对 y 坐标取反
		//	}

		//	// 使用ICP算法进行配准
		//	IterativeClosestPoint<PT, PT> icp;
		//	icp.setInputSource(cloud_mirror);
		//	icp.setInputTarget(more_cloud.makeShared());
		//	PointCloud<PT> Final;
		//	icp.align(Final);

		//	// 检查配准是否成功
		//	if (icp.hasConverged()) {
		//		std::cout << "ICP has converged, score is " << icp.getFitnessScore() << std::endl;
		//		std::cout << "Transformation matrix: \n" << icp.getFinalTransformation() << std::endl;

		//		// 将变换应用到镜像点云
		//		transformPointCloud(*cloud_mirror, *cloud_mirror, icp.getFinalTransformation());

		//		// 合并点云
		//		PointCloud<PointXYZ>::Ptr cloud_combined(new PointCloud<PointXYZ>());
		//		*cloud_combined = more_cloud + *cloud_mirror;

		//		//// 可视化合并后的点云
		//		//visualization::PCLVisualizer viewer("Cloud Viewer");
		//		//visualization::PointCloudColorHandlerCustom<PointXYZ> mirror_color(cloud_combined, 255, 0, 0);
		//		//viewer.addPointCloud<PointXYZ>(cloud_combined, mirror_color, "combined cloud");

		//		//while (!viewer.wasStopped()) {
		//		//	viewer.spinOnce();
		//		//}
		return 2;
	}
	//}
	return 1;
}

template<typename PT>
int isComplete(PointCloud<PointXYZ>::Ptr source_cloud) {
	//myVisualization<PT>(source_cloud, "origin tail");
	PT min{}, max{}, point{};
	typename PointCloud<PT>::Ptr hip_cloud(new PointCloud<PT>);
	getMinMax3D(*source_cloud, min, max);
	for (int i = 0; i < source_cloud->points.size(); i++) {
		if (source_cloud->points[i].x < min.x + 100)
		{
			point = source_cloud->points[i];
			point.z = 0;
			hip_cloud->points.push_back(point);
		}
	}

	// 通过凸包判断是否完整
	ConvexHull<PT> hull;
	hull.setInputCloud(hip_cloud);
	hull.setDimension(2); /*设置凸包维度*/
	std::vector<Vertices> polygons; /*用于保存凸包顶点*/
	typename PointCloud<PT>::Ptr surface_hull(new PointCloud<PT>);
	typename PointCloud<PT>::Ptr tail_hull(new PointCloud<PT>);
	hull.reconstruct(*surface_hull, polygons);

	getMinMax3D(*surface_hull, min, max);
	for (int i = 0; i < surface_hull->points.size(); i++) {
		if (surface_hull->points[i].x < max.x - 10) {
			tail_hull->points.push_back(surface_hull->points[i]);
		}
	}

	// 计算边界的曲率或拟合误差
	// 简单的方法是拟合二次曲线或多项式，然后计算拟合误差
	Eigen::MatrixXd A(tail_hull->points.size(), 3);
	Eigen::VectorXd b(tail_hull->points.size());
	for (size_t i = 0; i < tail_hull->points.size(); ++i) {
		A(i, 0) = tail_hull->points[i].y * tail_hull->points[i].y;
		A(i, 1) = tail_hull->points[i].y;
		A(i, 2) = 1.0;
		b(i) = tail_hull->points[i].x;
	}

	Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);
	double fit_error = (A * coeffs - b).norm();
	//cout << fit_error << " ";
	if (fit_error < 15) { return 1; }
	else { return 0; }

	int vertices_num = polygons[0].vertices.size();
	// Visualization
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1000.0);
	// Add the original point cloud
	visualization::PointCloudColorHandlerCustom<PointXYZ> cloud_color_handler(hip_cloud, 255, 255, 255);
	viewer->addPointCloud<PointXYZ>(hip_cloud, cloud_color_handler, "original cloud");
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original cloud");

	// Add the convex hull
	visualization::PointCloudColorHandlerCustom<PointXYZ> hull_color_handler(tail_hull, 255, 0, 0);
	viewer->addPointCloud<PointXYZ>(tail_hull, hull_color_handler, "hull cloud");
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "hull cloud");

	// 创建拟合曲线的点云
	typename PointCloud<PT>::Ptr fit_curve(new PointCloud<PT>);
	for (double y = min.y; y <= max.y; y += 1) { // 假设x范围在[-1, 1]
		double x = coeffs[0] * y * y + coeffs[1] * y + coeffs[2];
		fit_curve->points.push_back(PT(x, y, 0));
	}

	// 添加拟合曲线到可视化对象
	pcl::visualization::PointCloudColorHandlerCustom<PT> curve_color(fit_curve, 255, 0, 0);
	viewer->addPointCloud<PT>(fit_curve, curve_color, "fit curve");

	// Start the visualization loop
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	if (vertices_num < 35) {
		//cout << vertices_num;
		return 0;
	}
	return 1;
}
