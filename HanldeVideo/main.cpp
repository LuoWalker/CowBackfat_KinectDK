#include "GetPcd.h"
#include <iostream>
#include "AboutCamera.h"

using namespace std;

int main() {
	string filename = "03219.mkv";

	//video2Txt(filename, 11);
	//pyTxt2Pcd(filename);
	//PrintCalibrationFromFile(filename.c_str());
	int second[] = { 5,12,15,38 };
	int length = sizeof(second) / sizeof(second[0]);
	KinectRecord record(length);
	record.initRecord(filename, second);

	/*	0: 生成txt、转pcd、生成RGBD
		1：生成txt、转pcd
		2：转pcd
	*/
	record.getPCD(3);
	//record.getRGBD();

	return 0;
}