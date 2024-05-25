#include "GetFeature.hpp"
#include "GetObj.hpp"
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

void processPCD(const std::string& pcdFile, const std::string record_name, const std::string out_path, IMUSAMPLE imu_sample);

int main() {
	string record_path = "../PCD/origin/0520/";
	for (auto& v1 : directory_iterator(record_path)) {
		std::string record_name = v1.path().filename().string();
		std::string PCD_path = record_path + record_name + "/";
		std::string out_path = "../PCD/object/0520/" + record_name + "/";
		if (_access(out_path.c_str(), 0) == 0) { //判断该文件夹是否存在
			cout << "已跳过:" << record_name << endl;
			continue;
		}
		else {
			_mkdir(out_path.c_str());
		}
		// 文件路径

		vector<IMUSAMPLE> imu_samples;
		IMUSAMPLE imu_sample;
		std::string imu_filename = "../IMU/0520/" + record_name + ".txt";

		// 使用ifstream打开文件
		std::ifstream file(imu_filename);

		std::string line;
		// 遍历文件的每一行
		while (std::getline(file, line)) {
			// 处理每一行
			std::istringstream lineStream(line);
			for (int i = 0; i < 4; ++i) {
				if (i == 0) lineStream >> imu_sample.cur_frame;
				else if (i == 1) lineStream >> imu_sample.acc_x;
				else if (i == 2) lineStream >> imu_sample.acc_y;
				else if (i == 3) lineStream >> imu_sample.acc_z;
			}
			imu_samples.push_back(imu_sample);
		}

		// 文件结束时自动关闭
		file.close();
		std::vector<std::string> pcdFiles;
		for (auto& v2 : directory_iterator(PCD_path))
		{
			pcdFiles.push_back(v2.path().string());
		}

		// 启动多线程处理每个 txt 文件
		std::vector<std::thread> threads;
		cout << "开始处理：" << record_name << "\n";
		//myPointXYZ::Ptr template_cloud(new myPointXYZ);
		//io::loadPCDFile<PointXYZ>(PCD_path + "10008-t.pcd", *template_cloud);

		int n = 0;
		for (const auto& pcdFile : pcdFiles) {
			int pos1 = pcdFile.find("-");
			int pos2 = pcdFile.find(".");
			int cur_frame = stoi(pcdFile.substr(pos1 + 1, pos2 - pos1));
			if (cur_frame > imu_samples.size() + 5) continue;
			imu_sample = imu_samples[cur_frame - 6];
			//processPCD(pcdFile, record_name, out_path, imu_sample);

			threads.emplace_back([pcdFile, record_name, out_path, imu_sample]() {
				processPCD(pcdFile, record_name, out_path, imu_sample);
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

void processPCD(const std::string& pcdFile, const std::string record_name, const std::string out_path, IMUSAMPLE imu_sample) {
	std::string filename = path(pcdFile).filename().string();
	cout << pcdFile << "\n";

	// 读取点云
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

	//cout << "加载" << filename << "\n";
	io::loadPCDFile<PointXYZ>(pcdFile, *cloud);

	PointCloud<PointXYZ> cloud_xyz;
	PointCloud<PointNormal>::Ptr cloud_normal(new PointCloud<PointNormal>);
	PointCloud<PointXYZ>::Ptr cloud_line(new PointCloud<PointXYZ>);
	PointXYZ point;

	//cout << "归一化Z" << "\n";
	normalizationZ<PointXYZ>(cloud, imu_sample);
	//myVisualization<PointXYZ>(cloud, "Z");

	//cout << "连通域分割" << "\n";
	removeOtherObj<PointXYZ>(cloud);
	//myVisualization<PointXYZ>(cloud, "source");

	normalizationXOY<PointXYZ>(cloud, cloud_line);
	//myVisualization2<PointXYZ>(cloud, cloud_line, "maxz");

	//cout << "筛选" << "\n";
	bool is_filter = filterByVolume<PointXYZ>(cloud);
	if (!is_filter) {
		cout << "舍弃" << "\n";
		//myVisualization<PointXYZ>(cloud, "lvbo");
		return;
	}

	//cout << "下采样" << "\n";
	cloud_xyz = downSampleForce<PointXYZ>(cloud, 0.1);
	//myVisualization2<PointXYZ>(cloud, cloud_xyz.makeShared(), "downsample");

	*cloud = cloud_xyz;
	//cout << "平滑" << "\n";
	smoothByMLS<PointXYZ>(cloud, cloud_normal);
	//myVisualization2<PointXYZ>(cloud, cloud_normal, "smooth");

	copyPointCloud(*cloud_normal, *cloud);

	//cout << "裁切" << "\n";
	limitArea<PointXYZ>(cloud);
	//myVisualization<PointXYZ>(cloud, "Z");

	//cout << "连通域分割" << "\n";
	removeOtherObj<PointXYZ>(cloud);
	////cout << "归一化Z" << "\n";
	//normalizationZ<PointXYZ>(cloud);

	if (0 != _access((out_path).c_str(), 0)) {
		_mkdir((out_path).c_str());
	}
	io::savePCDFileASCII(out_path + filename, *cloud);
	//cout << "success" << "\n";
}