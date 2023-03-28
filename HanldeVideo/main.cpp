#include "GetPcd.h"
#include <iostream>
#include "AboutCamera.h"

using namespace std;

int main() {
	const char* video_path = "../Video/03212.mkv";

	video2Txt(video_path, 0);
	pyTxt2Pcd("1.txt", "test.pcd");
	PrintCalibrationFromFile(video_path);

	return 0;
}