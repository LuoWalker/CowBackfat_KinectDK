#pragma once
#define myPointXYZ pcl::PointCloud<pcl::PointXYZ>
#define myPointNormal pcl::PointCloud<pcl::PointNormal>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

myPointXYZ::Ptr transformByExtrinsics(myPointXYZ::Ptr target_cloud);
myPointXYZ::Ptr removeBackground(myPointXYZ::Ptr target_cloud, myPointXYZ::Ptr source_cloud);
myPointXYZ::Ptr removeNoise(myPointXYZ::Ptr target_cloud);
myPointNormal::Ptr smoothByMLS(myPointXYZ::Ptr target_cloud);