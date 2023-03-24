#include "GetPcd.h"
#include <iostream>

using namespace std;

int main() {
	const char* VideoPath = "../../Video/0321.mkv";

	//Video2Txt(VideoPath, 0);
	PyTxt2Pcd("1.txt", "test.pcd");

	return 0;
}