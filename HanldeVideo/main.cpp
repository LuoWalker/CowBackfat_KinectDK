#include "GetPcd.h"
#include "AboutCamera.h"
#include "pugixml.hpp"
#include <iostream>
#include <map>
#include <thread>
#include <vector>

using namespace std;
int main() {
	// 加载 XML 文件
	pugi::xml_document doc;
	if (!doc.load_file("recordInfo.xml")) {
		cerr << "Failed to load XML file." << endl;
		return 1;
	}

	// 创建一个 std::map 以存储数据
	map<string, int> recordInfo;
	string date = "0520";

	// 从 XML 中读取数据并存储到 map 中
	for (pugi::xml_node item : doc.child("info").find_child_by_attribute("data", "date", date.c_str()).children("item")) {
		std::string key = item.attribute("key").as_string();
		int value = item.text().as_int();
		recordInfo[key] = value;
	}

	// 逐个处理map中的记录
	string filename;
	int length = 0;
	int* second; //数组

	//Py py;
	for (const auto& pair : recordInfo) {
		filename = pair.first;
		length = pair.second;
		second = new int[length];
		for (int i = 0; i <= length; i++)
		{
			second[i] = i;
		}
		KinectRecord record(length);
		if (record.initRecord(date, filename, second) == 0) {
			continue;
		}
		/*	0: 生成txt、转pcd、生成RGBD
		1：生成txt、转pcd、输出IMU
		2：转pcd
		3: IMU
		*/
		record.getPCD(3);
		//record.getRGBD();
	}
	//py.closePy();
	return 0;
}