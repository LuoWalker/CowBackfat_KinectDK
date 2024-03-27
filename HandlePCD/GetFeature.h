#pragma once
#define myPointXYZ pcl::PointCloud<pcl::PointXYZ>
#define myPointXYZRGB pcl::PointCloud<pcl::PointXYZRGB>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string.h>

myPointXYZ::Ptr limitArea(myPointXYZ::Ptr target_cloud);
myPointXYZ::Ptr downSampleVoxelization(myPointXYZ::Ptr target_cloud);
myPointXYZ::Ptr getConvexHull(myPointXYZ::Ptr target_cloud);

myPointXYZRGB::Ptr limitArea(myPointXYZRGB::Ptr target_cloud);
myPointXYZRGB::Ptr downSampleVoxelization(myPointXYZRGB::Ptr target_cloud);
myPointXYZRGB::Ptr getConvexHull(myPointXYZRGB::Ptr target_cloud);
myPointXYZRGB::Ptr normalization(myPointXYZRGB::Ptr source_cloud);
myPointXYZRGB::Ptr normalizationZ(myPointXYZRGB::Ptr source_cloud);

Eigen::Matrix3f Schmidt_orthogonalization(Eigen::Matrix3f vector_group, int dimension, int quantity);
float scalar_product(Eigen::Matrix3f vector_group_1, Eigen::Matrix3f vector_group_2, int dimension, int a, int b);

bool isSymmetric(myPointXYZRGB::Ptr source_cloud);
myPointXYZRGB::Ptr segCloud(myPointXYZRGB::Ptr source_cloud);
void computeFPFH(pcl::PointCloud<pcl::FPFHSignature33>::Ptr& descriptors, myPointXYZRGB::Ptr cloud);