#include "GetPcd.h"
#include <iostream>
#include "AboutCamera.h"

using namespace std;

int main() {
	string filename = "0321.mkv";

	//video2Txt(filename, 0);
	pyTxt2Pcd(filename);
	//PrintCalibrationFromFile(filename.c_str());

	return 0;
}