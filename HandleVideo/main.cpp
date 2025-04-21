#include "GetPcd.h"
#include "pugixml.hpp"
#include <iostream>
#include <map>
#include <string>
#include <fstream>

using namespace std;
//int main() {
//	// 加载 XML 文件
//	pugi::xml_document doc;
//	if (!doc.load_file("recordInfo.xml")) {
//		cerr << "Failed to load XML file." << endl;
//		return 1;
//	}
//
//	// 创建一个 std::map 以存储数据
//	map<string, int> recordInfo;
//	string date = "0520";
//
//	// 从 XML 中读取数据并存储到 map 中
//	for (pugi::xml_node item : doc.child("info").find_child_by_attribute("data", "date", date.c_str()).children("item")) {
//		std::string key = item.attribute("key").as_string();
//		int value = item.text().as_int();
//		recordInfo[key] = value;
//	}
//
//	// 逐个处理map中的记录
//	string filename;
//	int length = 0;
//	int* second; //数组
//	for (const auto& pair : recordInfo) {
//		filename = pair.first;
//		length = pair.second;
//		second = new int[length];
//		for (int i = 0; i < length; i++)
//		{
//			second[i] = i;
//		}
//		KinectRecord record(length);
//		if (record.initRecord(date, filename, second) == 0) {
//			continue;
//		}
//		/*	0: 生成txt、转pcd、生成RGBD
//		1：生成txt、转pcd、输出IMU
//		2：转pcd
//		3: IMU
//		*/
//		record.getPCD(0);
//		//record.getRGBD();
//	}
//	//py.closePy();
//	return 0;
//}

//int main() {
//	// 打开录制文件
//	k4a_playback_t playback = NULL;
//	if (k4a_playback_open("F:\\luowenkuo\\IMUtest\\imu03192.mkv", &playback) != K4A_RESULT_SUCCEEDED) {
//		std::cerr << "无法打开录制文件！" << std::endl;
//		return 1;
//	}
//
//	// 获取文件信息
//	k4a_record_configuration_t config;
//	k4a_playback_get_record_configuration(playback, &config);
//	std::cout << "IMU 采样率: " << config.imu_track_enabled << std::endl;
//
//	// 循环读取 IMU 数据
//	k4a_stream_result_t result;
//	k4a_imu_sample_t imu_sample;
//
//	// 创建 CSV 文件并写入表头
//	std::ofstream outFile("D:\\Project_FatDepth\\KinectDK\\IMU\\test\\imuY.csv");
//	outFile << "acc_timestamp_usec,acc_x,acc_y,acc_z,gyro_timestamp_usec,gyro_x,gyro_y,gyro_z" << std::endl;
//
//	while (true) {
//		result = k4a_playback_get_next_imu_sample(playback, &imu_sample);
//		if (result == K4A_STREAM_RESULT_SUCCEEDED) {
//			// 写入 CSV 文件（时间戳单位：微秒，加速度单位：m/s²，陀螺仪单位：rad/s）
//			outFile << imu_sample.acc_timestamp_usec << ","
//				<< imu_sample.acc_sample.xyz.x << ","
//				<< imu_sample.acc_sample.xyz.y << ","
//				<< imu_sample.acc_sample.xyz.z << ","
//				<< imu_sample.gyro_timestamp_usec << ","
//				<< imu_sample.gyro_sample.xyz.x << ","
//				<< imu_sample.gyro_sample.xyz.y << ","
//				<< imu_sample.gyro_sample.xyz.z << std::endl;
//		}
//		else if (result == K4A_STREAM_RESULT_EOF) {
//			std::cout << "文件读取完成！" << std::endl;
//			break;
//		}
//		else {
//			std::cerr << "读取失败！" << std::endl;
//			break;
//		}
//	}
//	// 停止并关闭（需手动终止程序）
//	outFile.close();
//
//	// 关闭文件
//	k4a_playback_close(playback);
//	return 0;
//}

//int main() {
//	string date = "0111";
//	string filename = "10008";
//	int length = 13;
//	int* second = new int[length];
//
//	for (int i = 0; i < length; i++)
//	{
//		second[i] = i;
//	}
//
//	KinectRecord record(length);
//	if (record.initRecord(date, filename, second) == 1) {
//		/*	0: 生成txt、转pcd、生成RGBD
//		1：生成txt、转pcd、输出IMU
//		2：转pcd
//		3: IMU
//		*/
//		record.getPCD(3);
//		//record.getRGBD();
//	}
//	else {
//		cout << "error" << endl;
//	}
//
//	return 0;
//}

//int main() {
//	video2Txt("10008.mkv", 1);
//}