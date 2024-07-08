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
#include <pcl/sample_consensus/sac_model_line.h> // ���ֱ��
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

// ��������
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
	// desAxis: 0-x�ᣬ1-y�ᣬ2-z��
	// ��Ŀ������ת��Ϊ��λ����
	v.normalize();
	Eigen::Vector3d axis;
	double angle;
	switch (des_axis)
	{
	case 0:
		// ��������Ҫ��des_axis����ת������v��λ�ã����������Ҫ�ҵ���v��des_axis�����ת��
		// �����ͨ���������������Ĳ�����õ����������ֱ����תƽ��
		axis = v.cross(Eigen::Vector3d::UnitX());
		axis.normalize();
		// ������ת�Ƕȣ���������ʹ��acos������нǣ�������ת��Ϊ����
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

	// �����������
	Eigen::Affine3d transform = Eigen::Affine3d::Identity();
	PT min{}, max{};
	getMinMax3D(*source_cloud, min, max);
	//transform.translation() << -min.x, -min.y, -min.z; // ƽ��
	transform.translation() << 0, 0, 0; // ƽ��
	transform.rotate(Eigen::AngleAxisd(angle, axis)); // ��ת

	transformPointCloud(*source_cloud, *source_cloud, transform);
}

template<typename PT>
void rotationByVecotr(typename PointCloud<PT>::Ptr source_cloud, typename PointCloud<PT>::Ptr cloud_line, Eigen::Vector3d v, int des_axis) {
	// desAxis: 0-x�ᣬ1-y�ᣬ2-z��
	// ��Ŀ������ת��Ϊ��λ����
	v.normalize();
	Eigen::Vector3d axis;
	double angle;
	switch (des_axis)
	{
	case 0:
		// ��������Ҫ��des_axis����ת������v��λ�ã����������Ҫ�ҵ���v��des_axis�����ת��
		// �����ͨ���������������Ĳ�����õ����������ֱ����תƽ��
		axis = v.cross(Eigen::Vector3d::UnitX());
		axis.normalize();
		// ������ת�Ƕȣ���������ʹ��acos������нǣ�������ת��Ϊ����
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

	// �����������
	Eigen::Affine3d transform = Eigen::Affine3d::Identity();
	transform.translation() << 0, 0, 0; // ƽ��
	transform.rotate(Eigen::AngleAxisd(angle, axis)); // ��ת

	transformPointCloud(*source_cloud, *source_cloud, transform);
	transformPointCloud(*cloud_line, *cloud_line, transform);

	transform = Eigen::Affine3d::Identity();
	PT min{}, max{};
	getMinMax3D(*cloud_line, min, max);
	transform.translation() << -min.x, -min.y, -min.z; // ƽ��
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
	// ���㷨����������
	typename search::KdTree<PT>::Ptr kdtree(new search::KdTree<PT>);
	kdtree->setInputCloud(source_cloud);

	NormalEstimationOMP<PT, Normal> ne;
	ne.setInputCloud(source_cloud);
	ne.setSearchMethod(kdtree);
	ne.setKSearch(100);
	PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>);
	ne.compute(*cloud_normals);

	// ������ֵ
	double K1 = 0.5; // �н���ֵ�����Ը���ʵ�����������
	double K2 = 0.1; // ���ʾ�������ֵ�����Ը���ʵ�����������

	// �������ơ�������ʹ�ɾ���㼯
	typename PointCloud<PT>::Ptr cloud_keep(new PointCloud<PT>);
	typename PointCloud<PT>::Ptr cloud_filter(new PointCloud<PT>);
	typename PointCloud<PT>::Ptr cloud_delete(new PointCloud<PT>);

	std::vector<int> idx;
	std::vector<float> dis;
	Eigen::Vector3f current_normal;
	std::vector<Eigen::Vector3f> knn_normal;
	float normal_angle = 0.0;

	// �������е�
//#pragma omp parallel for 
	for (int i = 0; i < source_cloud->size(); ++i) {
		if (i % 1000 == 0)
			cout << i << " ";
		// ��ȡ��ǰ�㷨����
		current_normal = cloud_normals->points[i].getNormalVector3fMap();
		// ��ȡ��ǰ�������ڵķ�����
		kdtree->nearestKSearch(source_cloud->points[i], 21, idx, dis);
		for (int j = 1; j < idx.size(); ++j) {
			knn_normal.push_back(cloud_normals->points[idx[j]].getNormalVector3fMap());
		}
		// ���㵱ǰ�㷨�����������ڷ�������ֵ�ļн�
		normal_angle = calculateAngleA(current_normal, knn_normal);
		// �жϷ������н�A�Ƿ�С����ֵK1
		if (normal_angle < K1) {
			cloud_keep->push_back(source_cloud->points[i]);
			continue; // ������
		}
		else {
			// ��������㼯����
			std::vector<double> curvatures;
			for (const auto& idx : idx) {
				curvatures.push_back(calculateCurvature(source_cloud, { idx }));
			}

			// �������ʾ�ֵ
			double mean_curvature = std::accumulate(curvatures.begin(), curvatures.end(), 0.0) / curvatures.size();

			// �������ʾ������_D
			double sigma_D = calculateCurvatureStdDev(curvatures, mean_curvature);

			// �ж����ʾ������_D�Ƿ������ֵK2
			if (sigma_D > K2) {
				//#pragma omp critical
				cloud_filter->push_back(source_cloud->points[i]); // ���������㼯
			}
			else {
				//#pragma omp critical
				cloud_delete->push_back(source_cloud->points[i]); // �����ɾ���㼯
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
	// ���㷨����
	NormalEstimationOMP<PT, Normal> ne;
	ne.setInputCloud(source_cloud);
	typename search::KdTree<PT>::Ptr tree(new search::KdTree<PT>());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(30); // ���������뾶
	ne.compute(*normals);

	// ʹ�÷������ռ���������²���
	PointCloud<PT> sampled_cloud;
	NormalSpaceSampling<PT, Normal> nss;
	nss.setInputCloud(source_cloud);
	nss.setNormals(normals);
	nss.setBins(4, 4, 4); // ���÷������ռ��bin��
	nss.setSample(3000); // �����²����������
	nss.filter(sampled_cloud);
	return sampled_cloud;
}

template<typename PT>
PointCloud<PT> normalization(typename PointCloud<PT>::Ptr source_cloud) {

	// ��������
	Eigen::Vector4f centroid;
	compute3DCentroid(*source_cloud, centroid);

	// �����Ƽ�ȥ����
	PointCloud<PT> normalized_cloud;
	demeanPointCloud(*source_cloud, centroid, normalized_cloud);
	return normalized_cloud;
}

template<typename PT>
typename PointCloud<PT>::Ptr normalizationZ(typename PointCloud<PT>::Ptr source_cloud) {
	// ������϶���
	SACSegmentation<PT> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(SACMODEL_PLANE);
	seg.setMethodType(SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(170);

	seg.setInputCloud(source_cloud);

	// ����������ƽ��ϵ��
	PointIndices::Ptr inliers(new PointIndices());
	ModelCoefficients::Ptr coefficients(new ModelCoefficients());
	seg.segment(*inliers, *coefficients);

	// ��ȡ���ƽ���
	ExtractIndices<PT> extract;
	extract.setInputCloud(source_cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	typename PointCloud<PT>::Ptr back_plane(new PointCloud<PT>);
	extract.filter(*back_plane);

	// ���㷨��������Z��ļнǣ����õ���ת����
	Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
	float angle = acos(normal.dot(Eigen::Vector3f::UnitZ()));
	// ������ת�ı任����
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate(Eigen::AngleAxisf(angle, normal.cross(Eigen::Vector3f::UnitZ()).normalized()));

	// ����ϵõ���ƽ����ת����XOYƽ��ƽ��
	typename PointCloud<PT>::Ptr rotated_plane(new PointCloud<PT>);
	transformPointCloud(*back_plane, *rotated_plane, transform);
	return rotated_plane;
}

template<typename PT>
void normalizationZ(typename PointCloud<PT>::Ptr source_cloud, IMUSAMPLE imu_sample) {
	//// �������Ʋ�ȡ��Zֵ
	//for (auto& point : *source_cloud) {
	//	point.z = -point.z;
	//}
	// imu����������ϵ xyz
	// Eigen::Vector3d v = { imu_samples[1],imu_samples[2],imu_samples[3] };
	// �������ϵ
	Eigen::Vector3d v = { imu_sample.acc_y,-1.0 * imu_sample.acc_z,imu_sample.acc_x };
	// ��Z�����������ٶ���������
	rotationByVecotr<PT>(source_cloud, v, 2);

	// �������Ʋ�ȡ��Zֵ
	for (auto& point : *source_cloud) {
		point.z = -point.z;
	}
	limitZ<PT>(source_cloud);
}

template<typename PT>
void normalizationXOY(typename PointCloud<PT>::Ptr source_cloud, typename PointCloud<PT>::Ptr cloud_line) {
	// �������ƣ��ҵ����Zֵ�ĵ�
	PT min{}, max{};
	getMinMax3D(*source_cloud, min, max);
	int n = (max.x - min.x) / 10.0;
	PT max_point;
	typename PointCloud<PT>::Ptr maxz_cloud(new PointCloud<PT>);


	std::vector<double> intervals(n);
	for (int i = 0; i < n; ++i) {
		intervals[i] = min.x + (max.x * 0.7 - min.x) * i / (n - 1);
	}
	float maxZ = std::numeric_limits<float>::lowest();  // ��ʼ��Ϊ��С������ֵ
	for (int i = 0; i < intervals.size() - 1; i++) {
		//float minZ = std::numeric_limits<float>::max();  // ��ʼ��Ϊ��󸡵���ֵ
		float x1 = intervals[i];
		float x2 = intervals[i + 1];

		// ��������
		for (const auto& point : source_cloud->points) {
			// �����X�����Ƿ���ָ����Χ��
			if (point.x >= x1 && point.x <= x2) {
				// �������Zֵ��
				if (point.z > maxZ) {
					maxZ = point.z;
					max_point = point;
				}
			}
		}
		// �洢���Zֵ�㼰���Ӧ��X���귶Χ
		maxz_cloud->push_back(max_point);
		maxZ = std::numeric_limits<float>::lowest();
	}

	//-----------------------------���ֱ��-----------------------------
	typename SampleConsensusModelLine<PT>::Ptr model_line(new SampleConsensusModelLine<PT>(maxz_cloud));
	RandomSampleConsensus<PT> ransac(model_line);
	ransac.setDistanceThreshold(10);	//�ڵ㵽ģ�͵�������
	ransac.setMaxIterations(1000);		//����������
	ransac.computeModel();				//ֱ�����
	//--------------------------����������ȡ�ڵ�------------------------
	std::vector<int> inliers;
	ransac.getInliers(inliers);
	copyPointCloud<PT>(*maxz_cloud, inliers, *cloud_line);

	//----------------------------���ģ�Ͳ���--------------------------
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
	// �������Ʋ�ȡ��Zֵ
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
	//cout << "X����ֵ" << min.x << ' ' << max.x << endl;
	//cout << "Y����ֵ" << min.y << ' ' << max.y << endl;
	//cout << "Z����ֵ" << min.z << ' ' << max.z << endl;

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
	* 	�������ƣ��ҵ�Yֵ����������䣬Ϊ����
	*   �õ������Ҷ�Xֵ���Ե��ƽ��в���
	*/

	// Ԥ����
	PassThrough<PT> pass;
	PT min{}, max{};
	getMinMax3D(*source_cloud, min, max);
	//pass.setInputCloud(source_cloud);
	//pass.setFilterFieldName("z");
	//pass.setFilterLimits(max.z - 300, max.z);
	//pass.filter(*source_cloud);

	// ��������
	int max_width = 0;
	float gap = 10.0;
	int n = (max.x - min.x) / gap + 1;
	std::vector<double> intervals(n);
	for (int i = 0; i < n; ++i) {
		intervals[i] = min.x + i * gap;
	}

	// ͳ��ÿ������Ŀ��
	PointCloud<PT> bin_cloud;
	std::vector<int> width(n, 0);
	std::vector<PT> interval_min_y(n, { 0, std::numeric_limits<float>::max(), 0 });
	std::vector<PT> interval_max_y(n, { 0, std::numeric_limits<float>::min(), 0 });
	for (int i = 0; i < source_cloud->points.size(); i++) {
		// �ж����ĸ�����
		int n = (source_cloud->points[i].x - min.x) / gap;
		// ������ֵ��
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

	// �ҵ������������
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
		0:����
		1:���������辵��ֱ��ͨ��
		2:��������Ҫ���񣬷��ؾ����ĵ���
	*/
	PointCloud<PT> left_cloud, right_cloud, middle_cloud;
	PT min{}, max{};

	// ����������ƶ��̣ܶ�������POI��ֱ�Ӷ���
	getMinMax3D(*source_cloud, min, max);
	if ((max.x - min.x) < 500) {
		return 0;
	}

	// �Ѿ��������Ƶ�x����������������ֻ��Ҫ�ж�y�ᣬ�����Ϊ�������������ж�
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

	// �жϼ������ȣ���Ȼû�а취���о��������������Ҫ��500mm
	getMinMax3D(middle_cloud, min, max);
	if ((max.x - min.x) < 500) { return 0; }


	// �ų�������λ�쳣����������缹������������ͨ�����Zֵ��λ����ʱ�ᶨλ�����Ʊ߽�
	// ��ʱ�����ֻ�����һ���࣬��һ��㼫�٣�������Ҫ�ų��������
	getMinMax3D(right_cloud, min, max);
	if ((max.x - min.x) < 200 || (max.y - min.y) < 200) { return 0; }

	getMinMax3D(left_cloud, min, max);
	if ((max.x - min.x) < 200 || (max.y - min.y) < 200) { return 0; }

	// �ٵ�һ�������ж��Ƿ���Ҫ���񣬶��һ�����ھ���
	PointCloud<PT> less_cloud, more_cloud;
	less_cloud = left_cloud.points.size() > right_cloud.points.size() ? right_cloud : left_cloud;
	less_cloud = less_cloud + middle_cloud;
	more_cloud = left_cloud.points.size() > right_cloud.points.size() ? left_cloud : right_cloud;
	more_cloud = right_cloud + middle_cloud;
	if (less_cloud.points.size() < 0.5 * more_cloud.points.size()) {
		//	// ���ɾ������
		//	typename PointCloud<PT>::Ptr cloud_mirror(new PointCloud<PT>());
		//	*cloud_mirror = more_cloud;
		//	for (auto& point : cloud_mirror->points) {
		//		point.y = -point.y; // �� y ����ȡ��
		//	}

		//	// ʹ��ICP�㷨������׼
		//	IterativeClosestPoint<PT, PT> icp;
		//	icp.setInputSource(cloud_mirror);
		//	icp.setInputTarget(more_cloud.makeShared());
		//	PointCloud<PT> Final;
		//	icp.align(Final);

		//	// �����׼�Ƿ�ɹ�
		//	if (icp.hasConverged()) {
		//		std::cout << "ICP has converged, score is " << icp.getFitnessScore() << std::endl;
		//		std::cout << "Transformation matrix: \n" << icp.getFinalTransformation() << std::endl;

		//		// ���任Ӧ�õ��������
		//		transformPointCloud(*cloud_mirror, *cloud_mirror, icp.getFinalTransformation());

		//		// �ϲ�����
		//		PointCloud<PointXYZ>::Ptr cloud_combined(new PointCloud<PointXYZ>());
		//		*cloud_combined = more_cloud + *cloud_mirror;

		//		//// ���ӻ��ϲ���ĵ���
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

	// ͨ��͹���ж��Ƿ�����
	ConvexHull<PT> hull;
	hull.setInputCloud(hip_cloud);
	hull.setDimension(2); /*����͹��ά��*/
	std::vector<Vertices> polygons; /*���ڱ���͹������*/
	typename PointCloud<PT>::Ptr surface_hull(new PointCloud<PT>);
	typename PointCloud<PT>::Ptr tail_hull(new PointCloud<PT>);
	hull.reconstruct(*surface_hull, polygons);

	getMinMax3D(*surface_hull, min, max);
	for (int i = 0; i < surface_hull->points.size(); i++) {
		if (surface_hull->points[i].x < max.x - 10) {
			tail_hull->points.push_back(surface_hull->points[i]);
		}
	}

	// ����߽�����ʻ�������
	// �򵥵ķ�������϶������߻����ʽ��Ȼ�����������
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

	// ����������ߵĵ���
	typename PointCloud<PT>::Ptr fit_curve(new PointCloud<PT>);
	for (double y = min.y; y <= max.y; y += 1) { // ����x��Χ��[-1, 1]
		double x = coeffs[0] * y * y + coeffs[1] * y + coeffs[2];
		fit_curve->points.push_back(PT(x, y, 0));
	}

	// ���������ߵ����ӻ�����
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
