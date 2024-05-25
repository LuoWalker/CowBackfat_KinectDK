#pragma once
#include "GetFeature.h"
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <iostream>
#include <omp.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h> // ���ֱ��
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <vector>

//���ʼ���ṹ��
typedef struct CURVATURE {
	int index;
	float curvature;
}CURVATURE;


// ���㷨ʸ���н�A
static double calculateAngleA(const Eigen::Vector3f& normal, const std::vector<Eigen::Vector3f>& neighbor_normals) {
	double sum_dot_products = 0.0;
	for (const auto& neighbor_normal : neighbor_normals) {
		sum_dot_products += std::abs(normal.dot(neighbor_normal));
	}
	return sum_dot_products / neighbor_normals.size();
}

// ��������
static double calculateCurvature(const PointCloud<PointXYZ>::Ptr& cloud, const std::vector<int>& indices) {
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

// �������ʾ������_D
static double calculateCurvatureStdDev(const std::vector<double>& curvatures, double mean_curvature) {
	double sum = 0.0;
	for (const auto& curvature : curvatures) {
		sum += (curvature - mean_curvature) * (curvature - mean_curvature);
	}
	return std::sqrt(sum / curvatures.size());
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
	PT min, max;

	PassThrough<PT> pass;
	getMinMax3D(*source_cloud, min, max);

	pass.setInputCloud(source_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(max.z - 500, max.z);
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
PointCloud<PT> normalization(PointCloud<PT>& source_cloud) {

	// ��������
	Eigen::Vector4f centroid;
	compute3DCentroid(*source_cloud, centroid);

	// �����Ƽ�ȥ����
	PointCloud<PT> normalized_cloud(new PointCloud<PT>);
	demeanPointCloud(*source_cloud, centroid, *normalized_cloud);

	// ����Э�������
	Eigen::Matrix3f covariance_matrix;
	computeCovarianceMatrixNormalized(*normalized_cloud, centroid, covariance_matrix);

	// ��������ֵ����������
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
	Eigen::Vector3f eigen_values = eigen_solver.eigenvalues();
	Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();

	// ������ֵ��������������������
	std::vector<std::pair<double, Eigen::Vector3f>> eigen_pairs;
	for (int i = 0; i < eigen_values.size(); ++i) {
		eigen_pairs.push_back(std::make_pair(eigen_values(i), eigen_vectors.col(i)));
	}
	std::sort(eigen_pairs.begin(), eigen_pairs.end(), [](const auto& left, const auto& right) {
		return left.first > right.first;
		});
	for (int i = 0; i < eigen_values.size(); ++i) {
		eigen_values(i) = eigen_pairs[i].first;
		eigen_vectors.col(i) = eigen_pairs[i].second;
	}

	//cout << "����ֵ:\n" << eigen_values << "\n";
	//cout << "��������:\n" << eigen_vectors << "\n";
	//cout << "���ĵ�:\n" << centroid << "\n";

	// ����һ����λ������Ϊ�任����ĳ�ʼ��
	Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

	transform_matrix.block<3, 3>(0, 0) = eigen_vectors;
	transformPointCloud(*normalized_cloud, *normalized_cloud, transform_matrix);

	PT min{}, max{};
	getMinMax3D(*normalized_cloud, min, max);
	if (abs(min.x) > max.x) {
		Eigen::Affine3f new_transform_matrix = Eigen::Affine3f::Identity();
		// Z ������ת PI ����
		new_transform_matrix.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
		transformPointCloud(*normalized_cloud, *normalized_cloud, new_transform_matrix);
		cout << "��Z��ת��180" << "\n";
	}
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
	PT min, max;
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
void limitArea(typename PointCloud<PT>::Ptr source_cloud) {
	PT min, max;
	//cout << "X����ֵ" << min.x << ' ' << max.x << endl;
	//cout << "Y����ֵ" << min.y << ' ' << max.y << endl;
	//cout << "Z����ֵ" << min.z << ' ' << max.z << endl;

	PassThrough<PT> pass;
	getMinMax3D(*source_cloud, min, max);

	pass.setInputCloud(source_cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(min.x, min.x + 750);
	pass.filter(*source_cloud);

	//pass.setFilterFieldName("y");
	//pass.setFilterLimits(min.y, min.y + 750);
	//pass.filter(*result_cloud);

	pass.setInputCloud(source_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(max.z - 300, max.z);
	//pass.setFilterLimits(min.z, min.z + 500);
	pass.filter(*source_cloud);

	getMinMax3D(*source_cloud, min, max);
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << -min.x, -min.y, -min.z;
	transform.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));
	transformPointCloud(*source_cloud, *source_cloud, transform);
}

template<typename PT>
bool filterByVolume(typename PointCloud<PT>::Ptr source_cloud) {
	PointCloud<PT> left_cloud, right_cloud, middle_cloud;
	PT min{}, max{};

	getMinMax3D(*source_cloud, min, max);
	if ((max.x - min.x) < 500) {
		return false;
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

	// �жϼ������ȣ���Ȼû�а취���о��������������Ҫ��400mm
	getMinMax3D(middle_cloud, min, max);
	if ((max.x - min.x) < 500) {
		return false;
	}

	getMinMax3D(right_cloud, min, max);
	if ((max.x - min.x) < 200 || (max.y - min.y) < 200) {
		return false;
	}

	getMinMax3D(left_cloud, min, max);
	if ((max.x - min.x) < 200 || (max.y - min.y) < 200) {
		return false;
	}
	return true;
}

template<typename PT>
bool isSymmetric(typename PointCloud<PT>::Ptr source_cloud) {
	//typename PointCloud<PT>::Ptr xyz_cloud(new PointCloud<PT>);
	//copyPointCloud(*source_cloud, *xyz_cloud);
	//// Ϊ���ƹ��Ʒ���
	//NormalEstimation<PT, Normal> ne;
	//ne.setInputCloud(xyz_cloud);
	//typename search::KdTree<PT>::Ptr tree(typename new search::KdTree<PT>());
	//ne.setSearchMethod(tree);
	//PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
	//ne.setRadiusSearch(30); // ���÷��߹��ư뾶
	//ne.compute(*normals);

	//// ��������
	//PrincipalCurvaturesEstimation<PT, Normal, PrincipalCurvatures> pc;
	//pc.setInputCloud(xyz_cloud);
	//pc.setInputNormals(normals);
	//pc.setRadiusSearch(30);
	//PointCloud<PrincipalCurvatures>::Ptr principal_curvatures(new PointCloud<PrincipalCurvatures>);
	//pc.compute(*principal_curvatures);

	//std::vector<CURVATURE> curvatures;
	//CURVATURE curvature{};
	//float avg_curvature;
	//for (size_t i = 0; i < principal_curvatures->points.size(); ++i) {
	//	PrincipalCurvatures principal_curvature = principal_curvatures->points[i];
	//	avg_curvature = (principal_curvature.pc1 + principal_curvature.pc2) / 2.0;
	//	curvature.index = i;
	//	curvature.curvature = avg_curvature;
	//	curvatures.push_back(curvature);
	//}
	//std::sort(curvatures.begin(), curvatures.end(), [](CURVATURE a, CURVATURE b) {
	//	return a.curvature > b.curvature;
	//	});

	//typename PointCloud<PT>::Ptr selected_cloud(typenmae new PointCloud<PT>);
	//int n = 5000;
	//for (int i = 0; i < n; i++)
	//{
	//	selected_cloud->push_back(source_cloud->points[curvatures[i].index]);
	//}
	////myVisualization(selected_cloud, "selected");

	return true;
}

template<typename PT>
PointCloud<PT> segCloud(typename PointCloud<PT>::Ptr source_cloud) {

	cout << "1";
	// ����ģ��FPFH������
	PointCloud<FPFHSignature33>::Ptr tem_descriptors(new PointCloud<FPFHSignature33>);
	io::loadPCDFile("10008-fpfh.pcd", *tem_descriptors);
	cout << "1.1";
	// �������FPFH������
	PointCloud<FPFHSignature33>::Ptr source_descriptors(new PointCloud<FPFHSignature33>);
	computeFPFH(source_descriptors, source_cloud);

	cout << "2";
	// ����ƥ��
	registration::CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> est;
	est.setInputSource(source_descriptors);
	est.setInputTarget(tem_descriptors);
	CorrespondencesPtr correspondences(new Correspondences);
	est.determineCorrespondences(*correspondences);

	cout << "3";
	// �ָ�
	typename PointCloud<PT>::Ptr segmented_cloud(new PointCloud<PT>);
	ExtractIndices<PT> extract;
	//std::sort(correspondences->begin(), correspondences->end(), [](Correspondence a, Correspondence b) {
	//	return a.distance > b.distance;
	//	});

	for (const auto& correspondence : *correspondences)
	{
		if (correspondence.distance < 50)
			segmented_cloud->points.push_back(source_cloud->points[correspondence.index_query]);
	}
	segmented_cloud->width = segmented_cloud->points.size();
	segmented_cloud->height = 1;
	segmented_cloud->is_dense = true;

	return segmented_cloud;
}
