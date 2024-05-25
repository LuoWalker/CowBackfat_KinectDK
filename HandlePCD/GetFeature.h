#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string.h>
struct IMUSAMPLE
{
	int cur_frame;
	float acc_x;
	float acc_y;
	float acc_z;
};

using namespace pcl;