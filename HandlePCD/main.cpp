#include "GetFeature.h"
#include "GetObj.h"
#include <direct.h>
#include <filesystem>
#include <pcl/common/impl/io.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

using namespace std;
using std::filesystem::directory_iterator;
using std::filesystem::path;
void processPCD(const std::string& pcdFile, const std::string record_name, const std::string out_path);

int main() {
	//myPointXYZRGB::Ptr template_cloud(new myPointXYZRGB);
	//pcl::io::loadPCDFile("../PCD/origin/0111/10008/10008-t.pcd", *template_cloud);
	//template_cloud = downSampleVoxelization(template_cloud);
	//myVisualization(template_cloud, "down sample");
	//pcl::PointCloud<pcl::FPFHSignature33>::Ptr tem_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>);
	//computeFPFH(tem_descriptors, template_cloud);
	//pcl::io::savePCDFileASCII("10008-fpfh.pcd", *tem_descriptors);

	//processPCD("../PCD/origin/0111/10008/10008-100.pcd", "10008", "../PCD/object/0111/10008/");

	string record_path = "../PCD/origin/0111/";
	for (auto& v1 : directory_iterator(record_path)) {
		std::string record_name = v1.path().filename().string();
		std::string PCD_path = record_path + record_name + "/";
		std::string out_path = "../PCD/object/0111/" + record_name + "/";
		if (_access(out_path.c_str(), 0) == 0) { //判断该文件夹是否存在
			cout << "已跳过:" << record_name << endl;
			continue;
		}
		else {
			_mkdir(out_path.c_str());
		}
		std::vector<std::string> pcdFiles;
		for (auto& v2 : directory_iterator(PCD_path))
		{
			pcdFiles.push_back(v2.path().string());
		}

		// 启动多线程处理每个 txt 文件
		std::vector<std::thread> threads;
		cout << "开始处理：" << record_name << "\n";
		//myPointXYZRGB::Ptr template_cloud(new myPointXYZRGB);
		//pcl::io::loadPCDFile<pcl::PointXYZRGB>(PCD_path + "10008-t.pcd", *template_cloud);

		int n = 0;
		for (const auto& pcdFile : pcdFiles) {
			//processPCD(pcdFile, record_name);

			threads.emplace_back([pcdFile, record_name, out_path]() {
				processPCD(pcdFile, record_name, out_path);
				});

			n++;
			if (n % 6 == 0)
			{
				// 等待所有线程执行完毕
				for (auto& thread : threads) {
					thread.join();
				}
				threads.clear();
			}

		}
		// 等待所有线程执行完毕
		for (auto& thread : threads) {
			thread.join();
		}
		threads.clear();


		cout << "处理完成：" << record_name << "\n";
	}
	return 0;
}

void processPCD(const std::string& pcdFile, const std::string record_name, const std::string out_path) {
	std::string filename = path(pcdFile).filename().string();

	// 读取点云
	myPointXYZRGB::Ptr cloud(new myPointXYZRGB);
	//cout << "加载" << filename << "\n";
	pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcdFile, *cloud);

	//cout << "下采样" << "\n";
	cloud = downSampleVoxelization(cloud);

	//cout << "连通域分割" << "\n";
	cloud = removeOtherObj(cloud);
	//myVisualization(cloud, "source");

	//cout << "利用特征描述子分割点云" << "\n";
	//cloud = segCloud(cloud);
	//myVisualization(cloud, "seg");

	//cout << "归一化Z" << "\n";
	cloud = normalizationZ(cloud);
	//myVisualization(cloud, "Z");

	//cout << isSymmetric(cloud) << "\n";

	//cout << "裁切" << "\n";
	cloud = limitArea(cloud);

	//cout << "连通域分割" << "\n";
	cloud = removeOtherObj(cloud);
	//cout << "归一化Z" << "\n";
	cloud = normalizationZ(cloud);

	//myVisualization(cloud, "result");

	pcl::io::savePCDFileASCII(out_path + filename, *cloud);
	//cout << "success" << "\n";
}