#include "GetFeature.h"
#include "GetObj.h"
#include <cmath>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/filters/random_sample.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h> // 拟合直线
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <vector>


using namespace pcl;
using std::cout;
using std::endl;
using std::string;

//曲率计算结构体
typedef struct CURVATURE {
	int index;
	float curvature;
}CURVATURE;

myPointXYZ::Ptr limitArea(myPointXYZ::Ptr target_cloud) {

	myPointXYZ::Ptr result_cloud(new myPointXYZ);
	myPointXYZ::Ptr temp_cloud(new myPointXYZ);

	PointXYZ min, max;
	getMinMax3D(*target_cloud, min, max);

	PassThrough<PointXYZ> pass;
	//pass.setInputCloud(target_cloud);
	//pass.setFilterFieldName("z");
	//pass.setFilterLimits(min.z, min.z + 500);
	//pass.filter(*temp_cloud);

	//SampleConsensusModelLine<PointXYZ>::Ptr model_line(new SampleConsensusModelLine<PointXYZ>(temp_cloud));
	//RandomSampleConsensus<PointXYZ> ransac(model_line);
	//ransac.setDistanceThreshold(0.01);	//内点到模型的最大距离
	//ransac.setMaxIterations(1000);		//最大迭代次数
	//ransac.computeModel();

	//Eigen::VectorXf coef;
	//ransac.getModelCoefficients(coef);
	//cout << "直线方程为：\n"
	//	<< "   (x - " << coef[0] << ") / " << coef[3]
	//	<< " = (y - " << coef[1] << ") / " << coef[4]
	//	<< " = (z - " << coef[2] << ") / " << coef[5] << endl;

	//double theta = atan(coef[4] / coef[3]);
	//double theta = -0.717871;
	//cout << theta << endl;	//-0.717871
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << -min.x, -min.y, -min.z;
	transform.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));
	transformPointCloud(*target_cloud, *result_cloud, transform);

	//visualization::PCLVisualizer viewer(filename);
	//viewer.addCoordinateSystem(1000);
	//visualization::PointCloudColorHandlerCustom<PointXYZ> white(result_cloud, 255, 255, 255);
	//viewer.addPointCloud(result_cloud, white, "trans");

	getMinMax3D(*result_cloud, min, max);
	cout << "X轴最值" << min.x << ' ' << max.x << endl;
	cout << "Y轴最值" << min.y << ' ' << max.y << endl;
	cout << "Z轴最值" << min.z << ' ' << max.z << endl;

	pass.setInputCloud(result_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(max.z - 500, max.z);
	pass.filter(*result_cloud);

	//visualization::PointCloudColorHandlerCustom<PointXYZ> red(result_cloud, 255, 0, 0);
	//viewer.addPointCloud(result_cloud, red, "result");

	//while (!viewer.wasStopped()) {
	//	viewer.spinOnce(1000);
	//}

	return result_cloud;
}
myPointXYZ::Ptr downSampleVoxelization(myPointXYZ::Ptr source_cloud) {
	myPointXYZ::Ptr result_cloud(new myPointXYZ);
	VoxelGrid<PointXYZ> filter;
	filter.setInputCloud(source_cloud);
	filter.setLeafSize(10, 10, 10);
	filter.filter(*result_cloud);

	return result_cloud;
}
myPointXYZ::Ptr getConvexHull(myPointXYZ::Ptr target_cloud) {
	ConvexHull<PointXYZ> hull;
	hull.setInputCloud(target_cloud);
	hull.setDimension(3);

	std::vector<Vertices> polygons;
	myPointXYZ::Ptr surface_hull(new myPointXYZ);
	hull.reconstruct(*surface_hull, polygons);

	visualization::PCLVisualizer viewer("Cloud viewer");

	viewer.addPointCloud(target_cloud, "cloud");

	viewer.addPointCloud(surface_hull, "convex hull");
	viewer.addPolygon<PointXYZ>(surface_hull, 0, 0, 255, "polyline");
	//viewer.addPolygonMesh<PointXYZ>(surface_hull, polygons, "mesh");

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}

	return target_cloud;
}


myPointXYZRGB::Ptr downSampleVoxelization(myPointXYZRGB::Ptr source_cloud) {
	myPointXYZRGB::Ptr result_cloud(new myPointXYZRGB);

	// 随机下采样点云
	//pcl::RandomSample<PointXYZRGB> filter;
	//filter.setInputCloud(source_cloud);
	//filter.setSample(source_cloud->points.size() * 0.3);
	//filter.setSeed(1);

	// 体素下采样
	VoxelGrid<PointXYZRGB> filter;
	filter.setInputCloud(source_cloud);
	filter.setLeafSize(10, 10, 10);

	filter.filter(*result_cloud);

	return result_cloud;
}
myPointXYZRGB::Ptr normalization(myPointXYZRGB::Ptr source_cloud) {

	// 计算质心
	Eigen::Vector4f centroid;
	compute3DCentroid(*source_cloud, centroid);

	// 将点云减去质心
	myPointXYZRGB::Ptr normalized_cloud(new myPointXYZRGB);
	demeanPointCloud(*source_cloud, centroid, *normalized_cloud);

	// 构建协方差矩阵
	Eigen::Matrix3f covariance_matrix;
	computeCovarianceMatrixNormalized(*normalized_cloud, centroid, covariance_matrix);

	// 计算特征值和特征向量
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
	Eigen::Vector3f eigen_values = eigen_solver.eigenvalues();
	Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();

	// 对特征值进行排序并重排特征向量
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

	//cout << "特征值:\n" << eigen_values << "\n";
	//cout << "特征向量:\n" << eigen_vectors << "\n";
	//cout << "质心点:\n" << centroid << "\n";

	eigen_vectors = Schmidt_orthogonalization(eigen_vectors, 3, 3);
	//cout << "正交后特征向量:\n" << eigen_vectors << "\n";

	// 创建一个单位矩阵作为变换矩阵的初始化
	Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

	transform_matrix.block<3, 3>(0, 0) = eigen_vectors;
	transformPointCloud(*normalized_cloud, *normalized_cloud, transform_matrix);

	PointXYZRGB min, max;
	getMinMax3D(*normalized_cloud, min, max);
	if (abs(min.x) > max.x) {
		Eigen::Affine3f new_transform_matrix = Eigen::Affine3f::Identity();
		// Z 轴上旋转 PI 弧度
		new_transform_matrix.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
		transformPointCloud(*normalized_cloud, *normalized_cloud, new_transform_matrix);
		cout << "绕Z轴转了180" << "\n";
	}
	return normalized_cloud;
}
myPointXYZRGB::Ptr normalizationZ(myPointXYZRGB::Ptr source_cloud) {
	// 创建拟合对象
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(170);

	seg.setInputCloud(source_cloud);

	// 计算索引和平面系数
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	seg.segment(*inliers, *coefficients);

	// 提取拟合平面点
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(source_cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr back_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
	extract.filter(*back_plane);

	// 计算法线向量与Z轴的夹角，并得到旋转矩阵
	Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
	float angle = acos(normal.dot(Eigen::Vector3f::UnitZ()));
	// 构建旋转的变换矩阵
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate(Eigen::AngleAxisf(angle, normal.cross(Eigen::Vector3f::UnitZ()).normalized()));

	// 将拟合得到的平面旋转到与XOY平面平行
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*back_plane, *rotated_plane, transform);
	return rotated_plane;
}

Eigen::Matrix3f Schmidt_orthogonalization(Eigen::Matrix3f vector_group, int dimension, int quantity)
{
	Eigen::Matrix3f matrix;
	Eigen::Matrix3f result;
	int i, j, k;

	/*备份向量组。*/
	for (j = 0; j < quantity; j++)
		for (i = 0; i < dimension; i++)
			matrix(i, j) = vector_group(i, j);
	/*第一个向量取原向量组中的第一个向量，即不改变矩阵的第一列。对第二列向量开始循环。*/
	for (j = 1; j < quantity; j++)
		/*对向量的元素（坐标）依据Schimdt正交化方法进行逐个运算并输出。*/
		for (i = 0; i < dimension; i++)
		{
			/*先将原向量赋与新向量。*/
			vector_group(i, j) = matrix(i, j);
			/*按照Schmidt正交化方法让新向量化为依次与其他向量正交，就完成了正交化过程。*/
			for (k = 0; k < j; k++)
				vector_group(i, j) -= (scalar_product(matrix, vector_group, dimension, j, k) / scalar_product(vector_group, vector_group, dimension, k, k)) * vector_group(i, j);
		}
	result = vector_group;
	return result;
}
float scalar_product(Eigen::Matrix3f vector_group_1, Eigen::Matrix3f vector_group_2, int dimension, int a, int b)
{
	float result = 0;
	int i;

	for (i = 0; i < dimension; i++)
		result += vector_group_1(i, a) * vector_group_2(i, b);

	return result;
}

myPointXYZRGB::Ptr limitArea(myPointXYZRGB::Ptr source_cloud) {

	myPointXYZRGB::Ptr result_cloud(new myPointXYZRGB);


	PointXYZRGB min, max;
	getMinMax3D(*source_cloud, min, max);
	//cout << "X轴最值" << min.x << ' ' << max.x << endl;
	//cout << "Y轴最值" << min.y << ' ' << max.y << endl;
	//cout << "Z轴最值" << min.z << ' ' << max.z << endl;

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << -min.x, -min.y, -min.z;
	transform.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));
	transformPointCloud(*source_cloud, *result_cloud, transform);

	PassThrough<PointXYZRGB> pass;
	getMinMax3D(*result_cloud, min, max);

	pass.setInputCloud(result_cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(min.x, min.x + 750);
	pass.filter(*result_cloud);

	pass.setInputCloud(result_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(max.z - 500, max.z);
	pass.filter(*result_cloud);

	return result_cloud;
}

bool isSymmetric(myPointXYZRGB::Ptr source_cloud) {
	myPointXYZ::Ptr xyz_cloud(new myPointXYZ);
	copyPointCloud(*source_cloud, *xyz_cloud);
	// 为点云估计法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(xyz_cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(30); // 设置法线估计半径
	ne.compute(*normals);

	// 计算曲率
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
	pc.setInputCloud(xyz_cloud);
	pc.setInputNormals(normals);
	pc.setRadiusSearch(30);
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pc.compute(*principal_curvatures);

	std::vector<CURVATURE> curvatures;
	CURVATURE curvature{};
	float avg_curvature;
	for (size_t i = 0; i < principal_curvatures->points.size(); ++i) {
		pcl::PrincipalCurvatures principal_curvature = principal_curvatures->points[i];
		avg_curvature = (principal_curvature.pc1 + principal_curvature.pc2) / 2.0;
		curvature.index = i;
		curvature.curvature = avg_curvature;
		curvatures.push_back(curvature);
	}
	std::sort(curvatures.begin(), curvatures.end(), [](CURVATURE a, CURVATURE b) {
		return a.curvature > b.curvature;
		});

	myPointXYZRGB::Ptr selected_cloud(new myPointXYZRGB);
	int n = 5000;
	for (int i = 0; i < n; i++)
	{
		selected_cloud->push_back(source_cloud->points[curvatures[i].index]);
	}
	myVisualization(selected_cloud, "selected");

	return true;
}

myPointXYZRGB::Ptr segCloud(myPointXYZRGB::Ptr source_cloud) {

	cout << "1";
	// 加载模板FPFH描述子
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr tem_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::io::loadPCDFile("10008-fpfh.pcd", *tem_descriptors);
	cout << "1.1";
	// 计算点云FPFH描述子
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>);
	computeFPFH(source_descriptors, source_cloud);

	cout << "2";
	// 特征匹配
	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
	est.setInputSource(source_descriptors);
	est.setInputTarget(tem_descriptors);
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
	est.determineCorrespondences(*correspondences);

	cout << "3";
	// 分割
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	//std::sort(correspondences->begin(), correspondences->end(), [](pcl::Correspondence a, pcl::Correspondence b) {
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

void  computeFPFH(pcl::PointCloud<pcl::FPFHSignature33>::Ptr& descriptors, myPointXYZRGB::Ptr cloud) {
	// 计算法线
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(20); // 设置搜索半径
	ne.compute(*normals);

	// 计算点云FPFH描述子
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
	fpfh_estimation.setInputCloud(cloud);
	fpfh_estimation.setInputNormals(normals);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	fpfh_estimation.setSearchMethod(kdtree);
	fpfh_estimation.setRadiusSearch(30); // 设置搜索半径
	fpfh_estimation.compute(*descriptors);
}
