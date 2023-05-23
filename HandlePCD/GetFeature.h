#pragma once
#define myPointXYZ pcl::PointCloud<pcl::PointXYZ>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string.h>

myPointXYZ::Ptr limitArea(myPointXYZ::Ptr target_cloud, std::string filename);
myPointXYZ::Ptr downSampleVoxelization(myPointXYZ::Ptr target_cloud);
myPointXYZ::Ptr getConvexHull(myPointXYZ::Ptr target_cloud);
