#include "GetPcd.h"
#include <iostream>
#include "AboutCamera.h"

using namespace std;

int main() {
	string filename = "051317.mkv";

	//video2Txt(filename, 11);
	//pyTxt2Pcd(filename);
	//PrintCalibrationFromFile(filename.c_str());
	int second[] = { 4,19,40,49,67,85 };
	int length = sizeof(second) / sizeof(second[0]);
	KinectRecord record(length);
	record.initRecord(filename, second);
	record.getPCD();

	return 0;
}