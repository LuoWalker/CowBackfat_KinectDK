#pragma once
#define myPointXYZ pcl::PointCloud<pcl::PointXYZ>
#define myPointXYZRGB pcl::PointCloud<pcl::PointXYZRGB>
#define myPointNormal pcl::PointCloud<pcl::PointNormal>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

myPointXYZ::Ptr transformByExtrinsics(myPointXYZ::Ptr target_cloud);
myPointXYZ::Ptr removeBackground(myPointXYZ::Ptr target_cloud, myPointXYZ::Ptr source_cloud);
myPointXYZ::Ptr removeNoise(myPointXYZ::Ptr target_cloud);
myPointXYZ::Ptr removeOtherObj(myPointXYZ::Ptr target_cloud);
myPointNormal::Ptr smoothByMLS(myPointXYZ::Ptr target_cloud);
myPointXYZ::Ptr getObj(myPointXYZ::Ptr target_cloud, myPointXYZ::Ptr source_cloud);
//void myVisualization(myPointXYZ::Ptr cloud);

myPointXYZRGB::Ptr transformByExtrinsics(myPointXYZRGB::Ptr target_cloud);
myPointXYZRGB::Ptr removeBackground(myPointXYZRGB::Ptr target_cloud, myPointXYZRGB::Ptr source_cloud);
myPointXYZRGB::Ptr removeNoise(myPointXYZRGB::Ptr target_cloud);
myPointXYZRGB::Ptr removeOtherObj(myPointXYZRGB::Ptr target_cloud);
myPointNormal::Ptr smoothByMLS(myPointXYZRGB::Ptr target_cloud);
myPointXYZRGB::Ptr getObj(myPointXYZRGB::Ptr target_cloud, myPointXYZRGB::Ptr source_cloud);
void myVisualization(myPointXYZRGB::Ptr cloud, const char*);