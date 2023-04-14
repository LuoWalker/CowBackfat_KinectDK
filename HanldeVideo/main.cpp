#include "GetPcd.h"
#include <iostream>
#include "AboutCamera.h"

using namespace std;

int main() {
	string filename = "03218.mkv";

	video2Txt(filename, 0);
	//pyTxt2Pcd("1.txt", "test.pcd");
	//PrintCalibrationFromFile(filename.c_str());

	return 0;
}