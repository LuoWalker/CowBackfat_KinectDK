#pragma once
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr transformByExtrinsics(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr removeBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr removeNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);
pcl::PointCloud<pcl::PointNormal>::Ptr smoothByMLS(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);