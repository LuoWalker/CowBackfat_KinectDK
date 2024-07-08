#include "GetFeature.hpp"
#include "GetObj.hpp"
#include <array>
#include <direct.h>
#include <filesystem>
#include <io.h>
#include <iosfwd>
#include <iostream>
#include <iterator>
#include <pcl/common/impl/io.hpp>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

using namespace std;
using std::filesystem::directory_iterator;
using std::filesystem::path;

void processPCD(const std::string& pcdFile, const std::string record_name, const std::string out_path, IMUSAMPLE imu_sample);

int main() {
	string date = "0614";
	string record_path = "F:/luowenkuo/PCD/origin/" + date + "/";
	for (auto& v1 : directory_iterator(record_path)) {
		std::string record_name = v1.path().filename().string();
		std::string PCD_path = record_path + record_name + "/";
		std::string out_path = "F:/luowenkuo/PCD/object/" + date + "test/" + record_name + "/";
		if (_access(out_path.c_str(), 0) == 0) { //�жϸ��ļ����Ƿ����
			cout << "������:" << record_name << endl;
			continue;
		}

		vector<IMUSAMPLE> imu_samples;
		IMUSAMPLE imu_sample;
		std::string imu_filename = "../IMU/" + date + "/" + record_name + ".txt";

		// ʹ��ifstream���ļ�
		std::ifstream file(imu_filename);

		std::string line;
		// �����ļ���ÿһ��
		while (std::getline(file, line)) {
			// ����ÿһ��
			std::istringstream lineStream(line);
			for (int i = 0; i < 4; ++i) {
				if (i == 0) lineStream >> imu_sample.cur_frame;
				else if (i == 1) lineStream >> imu_sample.acc_x;
				else if (i == 2) lineStream >> imu_sample.acc_y;
				else if (i == 3) lineStream >> imu_sample.acc_z;
			}
			imu_samples.push_back(imu_sample);
		}

		// �ļ�����ʱ�Զ��ر�
		file.close();
		std::vector<std::string> pcdFiles;
		for (auto& v2 : directory_iterator(PCD_path))
		{
			pcdFiles.push_back(v2.path().string());
		}


		cout << "��ʼ����" << record_name << "\n";
		int n = 0;
		int total_files = pcdFiles.size();

#pragma omp parallel for schedule(dynamic) reduction(+:n)
		for (int i = 0; i < total_files; ++i) {
			const auto& pcdFile = pcdFiles[i];
			int pos1 = pcdFile.find_last_of("-");
			int pos2 = pcdFile.find(".");
			int cur_frame = stoi(pcdFile.substr(pos1 + 1, pos2 - pos1));
			if (cur_frame > imu_samples.size() + 5) continue;
			imu_sample = imu_samples[cur_frame - 6];

			{
				//std::vector<string> pcdFiles{ "F:/luowenkuo/PCD/origin/0609/1132/1132-129.pcd",
				//	"F:/luowenkuo/PCD/origin/0609/1035/1035-22.pcd",
				//	"F:/luowenkuo/PCD/origin/0609/1035/1035-148.pcd",
				//	"F:/luowenkuo/PCD/origin/0609/1035/1035-416.pcd",
				//	"F:/luowenkuo/PCD/origin/0609/1132/1132-32.pcd",
				//	"F:/luowenkuo/PCD/origin/0609/1132/1132-369.pcd",
				//	"F:/luowenkuo/PCD/origin/0609/1218/1218-346.pcd",
				//	"F:/luowenkuo/PCD/origin/0609/1494/1494-42.pcd",
				//	"F:/luowenkuo/PCD/origin/0609/19009/19009-404.pcd"
				//};
			}

			processPCD(pcdFile, record_name, out_path, imu_sample);
			n++;

			if (n % 30 == 0) {
#pragma omp critical
				{
					std::cout << n / 30 << " ";
				}
			}
		}
		cout << "������ɣ�" << record_name << "\n";
	}

	return 0;
	{
		//string date = "0520";
		//string pcd_folder = "F:/luowenkuo/PCD/pcd" + date + "/";
		//IMUSAMPLE imu_sample = { 6,0,0,0 };
		//int n = 0;
		////�������̴߳���ÿ���ļ�
		//std::vector<std::thread> threads;

		//for (auto& v1 : directory_iterator(pcd_folder)) {
		//	std::string record_name = "111";
		//	std::string filename = v1.path().filename().string();
		//	std::string PCD_path = pcd_folder + filename;
		//	std::string out_path = "F:/luowenkuo/PCD/newpcd" + date + "/";
		//	//if (_access(out_path.c_str(), 0) == 0) { //�жϸ��ļ����Ƿ����
		//	//	cout << "������:" << record_name << endl;
		//	//	continue;
		//	//}

		//	threads.emplace_back([PCD_path, record_name, out_path, imu_sample]() {
		//		processPCD(PCD_path, record_name, out_path, imu_sample);
		//		});
		//	n++;
		//	if (n % 6 == 0)
		//	{
		//		// �ȴ������߳�ִ�����
		//		for (auto& thread : threads) {
		//			thread.join();
		//		}
		//		threads.clear();
		//	}

		//}
		//// �ȴ������߳�ִ�����
		//for (auto& thread : threads) {
		//	thread.join();
		//}
		//threads.clear();
		//return 0;
	}
}


void processPCD(const std::string& pcdFile, const std::string record_name, const std::string out_path, IMUSAMPLE imu_sample) {
	std::string filename = path(pcdFile).filename().string();

	// ��ȡ����
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	//clock_t start, end;
	//start = clock();
	//cout << "����" << filename << "-";
	io::loadPCDFile<PointXYZ>(pcdFile, *cloud);
	//io::loadPCDFile<PointXYZ>("F:/luowenkuo/PCD/origin/0614/1257/1257-221.pcd", *cloud);
	//myVisualization<PointXYZ>(cloud, "origin");
	//end = clock();
	//cout << (double)(end - start) / CLOCKS_PER_SEC << "\n";
	PointCloud<PointXYZ> cloud_xyz;
	PointCloud<PointNormal>::Ptr cloud_normal(new PointCloud<PointNormal>);
	PointCloud<PointXYZ>::Ptr cloud_line(new PointCloud<PointXYZ>);
	PointXYZ point;
	int flag = 0;
	//goto mls;

	////cout << "��һ��Z" << "\n";
	//normalizationZ<PointXYZ>(cloud, imu_sample);
	//myVisualization<PointXYZ>(cloud, "norm Z");

	//start = clock();
	//cout << "�²���" << "-";
	*cloud = downSampleForce<PointXYZ>(cloud, 0.5);
	//end = clock();
	//cout << (double)(end - start) / CLOCKS_PER_SEC << "\n";
	//myVisualization2<PointXYZ>(cloud, cloud_xyz.makeShared(), "downsample");
	//myVisualization<PointXYZ>(cloud, "downsample");

	//start = clock();
	//cout << "Ԥ����-Z�������и��С��������" << "-";
	preProcess<PointXYZ>(cloud);
	//end = clock();
	//cout << (double)(end - start) / CLOCKS_PER_SEC << "\n";
	//myVisualization<PointXYZ>(cloud, "pre");
	if (cloud->size() < 1000) {
		//cout << "����" << "\n";
		return;
	}

	//start = clock();
	//cout << "��ͨ��ָ�" << "-";
	removeOtherObj<PointXYZ>(cloud, 1);
	//end = clock();
	//cout << (double)(end - start) / CLOCKS_PER_SEC << "\n";
	//myVisualization<PointXYZ>(cloud, "source");
	//io::savePCDFileASCII(out_path + filename, *cloud);

	//��Ҫ�ҵ�ÿһ�����Zֵ��������Ҫ�ȶԵ��ƽ����²�������ͨ��ָ�
	//start = clock();
	//cout << "��һ��XOY" << "-";
	normalizationXOY<PointXYZ>(cloud, cloud_line);
	//end = clock();
	//cout << (double)(end - start) / CLOCKS_PER_SEC << "\n";
	//myVisualization2<PointXYZ>(cloud, cloud_line, "maxz");

	if (0 != _access((out_path).c_str(), 0)) {
		_mkdir((out_path).c_str());
	}
	if (0 != _access((out_path + "drop/").c_str(), 0)) {
		_mkdir((out_path + "drop/").c_str());
	}
	//cout << "����͹�������ж��Ƿ�����" << "\n";
	flag = isComplete<PointXYZ>(cloud);
	if (flag == 0) {
		//cout << "����" << "\n";
		io::savePCDFileBinary(out_path + "drop/1-" + filename, *cloud);
		//goto End;
		return;
	}

	//start = clock();
	//cout << "���� ��λ����" << "-";
	limitArea<PointXYZ>(cloud);
	//��λ���ǣ���β��
	keyPointPosition<PointXYZ>(cloud);
	//end = clock();
	//cout << (double)(end - start) / CLOCKS_PER_SEC << "\n";
	//myVisualization<PointXYZ>(cloud, "X");

	//start = clock();
	//cout << "ɸѡ" << "-";
	flag = filterByVolume<PointXYZ>(cloud);
	if (flag == 0) {
		//cout << "����" << "\n";
		//myVisualization<PointXYZ>(cloud, "drop");
		io::savePCDFileBinary(out_path + "drop/2-" + filename, *cloud);
		//std::string png_out_path = out_path;

		//size_t pos = png_out_path.find("object");
		//if (pos != std::string::npos) {
		//	png_out_path.replace(pos, 6, "preview");
		//}
		//if (0 != _access((png_out_path).c_str(), 0)) {
		//	_mkdir((png_out_path).c_str());
		//}
		//size_t dotPos = filename.rfind('.');
		//if (dotPos != std::string::npos) {
		//	filename.substr(0, dotPos) + ".png";
		//}
		//saveScreenshot<PointXYZ>(cloud, png_out_path + filename);
		//goto End;
		return;
	}
	//end = clock();
	//cout << (double)(end - start) / CLOCKS_PER_SEC << "\n";

	//start = clock();
	//cout << "ͳ���˲�" << "-";
	removeNoise<PointXYZ>(cloud);
	//end = clock();
	//cout << (double)(end - start) / CLOCKS_PER_SEC << "\n";
	//myVisualization<PointXYZ>(cloud, "denoise");

	//start = clock();
	//cout << "��ͨ��ָ�" << "-";
	removeOtherObj<PointXYZ>(cloud, 0);
	//end = clock();
	//cout << (double)(end - start) / CLOCKS_PER_SEC << "\n";
mls:
	//cout << "ƽ��" << "\n";
	//smoothByMLS<PointXYZ>(cloud, cloud_normal);
	//copyPointCloud(*cloud_normal, *cloud);
	//myVisualization<PointXYZ>(cloud, "smooth");

	//cout << "���߹���" << "\n";

	//start = clock();
	//cout << "��һ��" << "-";
	//ƽ�Ƶ�����
	//*cloud_normal = normalization<PointNormal>(cloud_normal);
	*cloud = normalization<PointXYZ>(cloud);
	//myVisualization<PointNormal>(cloud_normal, "result");
	//end = clock();
	//cout << (double)(end - start) / CLOCKS_PER_SEC << "\n";

	//start = clock();
	//cout << "����" << "-";
	std::string png_out_path = out_path;
	if (flag == 2) {
		io::savePCDFileBinary(out_path + "M" + filename, *cloud);
	}
	io::savePCDFileBinary(out_path + filename, *cloud);
	//end = clock();
	//cout << (double)(end - start) / CLOCKS_PER_SEC << "\n";
	//End:
	//	end = clock();
	//	cout << (double)(end - start) / CLOCKS_PER_SEC << "\n";
		//cout << "success" << "\n";
}