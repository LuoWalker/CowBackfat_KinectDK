#include "GetPcd.h"
#include <iostream>
#include "AboutCamera.h"

using namespace std;

int main() {
	string filename = "03213.mkv";

	video2Txt(filename, 1);
	pyTxt2Pcd(filename);
	//PrintCalibrationFromFile(filename.c_str());
	//KinectRecord record;
	//record.initRecord(filename, 0.2);
	//record.getPointCloud();


	return 0;
}