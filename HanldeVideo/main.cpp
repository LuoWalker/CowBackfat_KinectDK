#include "GetPcd.h"
#include <iostream>
#include "AboutCamera.h"

using namespace std;

int main() {
	string filename = "05139.mkv";

	//video2Txt(filename, 11);
	//pyTxt2Pcd(filename);
	//PrintCalibrationFromFile(filename.c_str());
	int second[] = { 2,3,29 };
	int length = sizeof(second) / sizeof(second[0]);
	KinectRecord record(length);
	record.initRecord(filename, second);
	record.getPCD();

	return 0;
}