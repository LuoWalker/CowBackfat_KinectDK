#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <filesystem>
#include "GetObj.h"
#include "GetFeature.h"

using std::experimental::filesystem::v1::directory_iterator;
using std::experimental::filesystem::v1::path;

int main() {
	// 读取点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (auto &v : directory_iterator("../PCD/origin/1123"))
	{
		std::string filename = v.path().filename().string();
		cout << "../PCD/origin/1123/" + filename << "\n";

		cout << "加载PCD" << "\n";
		pcl::io::loadPCDFile<pcl::PointXYZ>("../PCD/origin/1123/" + filename, *cloud);

		cout << "下采样" << "\n";
		cloud = downSampleVoxelization(cloud);

		cout << "连通域分割" << "\n";
		cloud = removeOtherObj(cloud);

		cout << "保存" << "\n";
		pcl::io::savePCDFileASCII("../PCD/object/1123/" + filename, *cloud);
		cout << "success" << "\n";
	}
	return 0;
}
