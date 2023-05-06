#define myPointXYZ pcl::PointCloud<pcl::PointXYZ>
#include "GetFeature.h"
#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h> // ���ֱ��
#include <pcl/surface/convex_hull.h>

using namespace pcl;
using std::cout;
using std::endl;
using std::string;

myPointXYZ::Ptr limitArea(myPointXYZ::Ptr target_cloud) {

	PointXYZ min, max;
	getMinMax3D(*target_cloud, min, max);

	myPointXYZ::Ptr result_cloud(new myPointXYZ);
	myPointXYZ::Ptr temp_cloud(new myPointXYZ);

	PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(target_cloud);
	pass.setFilterFieldName("z");
	cout << "Z����Сֵ" << min.z << ' ' << "Z�����ֵ" << max.z << endl;
	pass.setFilterLimits(max.z - 300, max.z);
	pass.filter(*result_cloud);

	getMinMax3D(*result_cloud, min, max);
	pass.setInputCloud(result_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(max.z - 10, max.z);
	pass.filter(*temp_cloud);

	SampleConsensusModelLine<PointXYZ>::Ptr model_line(new SampleConsensusModelLine<PointXYZ>(temp_cloud));
	RandomSampleConsensus<PointXYZ> ransac(model_line);
	ransac.setDistanceThreshold(0.01);	//�ڵ㵽ģ�͵�������
	ransac.setMaxIterations(1000);		//����������
	ransac.computeModel();

	Eigen::VectorXf coef;
	ransac.getModelCoefficients(coef);
	cout << "ֱ�߷���Ϊ��\n"
		<< "   (x - " << coef[0] << ") / " << coef[3]
		<< " = (y - " << coef[1] << ") / " << coef[4]
		<< " = (z - " << coef[2] << ") / " << coef[5] << endl;

	double theta = atan(coef[4] / coef[3]);
	cout << theta << endl;
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
	transform.translation() << coef[0], 0, 0;
	transformPointCloud(*result_cloud, *result_cloud, transform);

	getMinMax3D(*result_cloud, min, max);
	pass.setInputCloud(result_cloud);
	pass.setFilterFieldName("y");
	cout << "Y����Сֵ" << min.y << ' ' << "Y�����ֵ" << max.y << endl;
	pass.setFilterLimits(min.y, min.y + 1000);
	pass.filter(*result_cloud);

	visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addCoordinateSystem(1000);
	visualization::PointCloudColorHandlerCustom<PointXYZ> red(result_cloud, 255, 255, 255);
	viewer.addPointCloud(result_cloud, red, "result");
	//visualization::PointCloudColorHandlerCustom<PointXYZ> white(target_cloud, 255, 255, 255);
	//viewer.addPointCloud(target_cloud, white, "target");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return result_cloud;
}

myPointXYZ::Ptr downSampleVoxelization(myPointXYZ::Ptr source_cloud) {
	myPointXYZ::Ptr result_cloud(new myPointXYZ);
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(source_cloud);
	filter.setLeafSize(5, 5, 5);
	filter.filter(*result_cloud);

	visualization::PCLVisualizer viewer("Cloud Viewer");     //����viewer����
	int v1(0), v2(1);

	//ԭʼ����
	viewer.createViewPort(0, 0, 0.5, 1, v1);	// �Խ������꣨x1,y1,x2,y2��
	viewer.setBackgroundColor(0, 0, 0, v1);
	visualization::PointCloudColorHandlerCustom<PointXYZ> white1(source_cloud, 255, 255, 255);
	viewer.addPointCloud(source_cloud, white1, "source cloud", v1);	// Ϊ�����Զ�����ɫΪ��ɫ
	cout << source_cloud->points.size() << endl;

	//���ػ���
	viewer.createViewPort(0.5, 0, 1, 1, v2);
	viewer.setBackgroundColor(0, 0, 0, v2);
	visualization::PointCloudColorHandlerCustom<PointXYZ> white2(result_cloud, 255, 255, 255);
	viewer.addPointCloud(result_cloud, white2, "result_cloud", v2);	// Ϊ�����Զ�����ɫΪ��ɫ	


	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}	// ʹ����ͣ��

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
	viewer.addPolygonMesh<PointXYZ>(surface_hull, polygons, "mesh");

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}

	return target_cloud;
}