#include "AboutCamera.h"
#include <cstdio>

void PrintCalibrationFromFile(const char* path) {
	//输出相机参数
	k4a_playback_t handle;
	k4a_calibration_t calibration;

	k4a_playback_open(path, &handle);
	if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(handle, &calibration))
	{
		printf("Failed to get calibration\n");
	}
	auto calib = calibration.depth_camera_calibration;
	cout << "深度相机内参：" << endl;
	cout << "resolution width: " << calib.resolution_width << endl;
	cout << "resolution height: " << calib.resolution_height << endl;
	cout << "principal point x: " << calib.intrinsics.parameters.param.cx << endl;
	cout << "principal point y: " << calib.intrinsics.parameters.param.cy << endl;
	cout << "focal length x: " << calib.intrinsics.parameters.param.fx << endl;
	cout << "focal length y: " << calib.intrinsics.parameters.param.fy << endl;
	cout << "radial distortion coefficients:" << endl;
	cout << "k1: " << calib.intrinsics.parameters.param.k1 << endl;
	cout << "k2: " << calib.intrinsics.parameters.param.k2 << endl;
	cout << "k3: " << calib.intrinsics.parameters.param.k3 << endl;
	cout << "k4: " << calib.intrinsics.parameters.param.k4 << endl;
	cout << "k5: " << calib.intrinsics.parameters.param.k5 << endl;
	cout << "k6: " << calib.intrinsics.parameters.param.k6 << endl;
	cout << "center of distortion in Z=1 plane, x: " << calib.intrinsics.parameters.param.codx << endl;
	cout << "center of distortion in Z=1 plane, y: " << calib.intrinsics.parameters.param.cody << endl;
	cout << "tangential distortion coefficient x: " << calib.intrinsics.parameters.param.p1 << endl;
	cout << "tangential distortion coefficient y: " << calib.intrinsics.parameters.param.p2 << endl;
	cout << "metric radius: " << calib.intrinsics.parameters.param.metric_radius << endl;
	calib = calibration.color_camera_calibration;
	cout << "彩色相机内参：" << endl;
	cout << "resolution width: " << calib.resolution_width << endl;
	cout << "resolution height: " << calib.resolution_height << endl;
	cout << "principal point x: " << calib.intrinsics.parameters.param.cx << endl;
	cout << "principal point y: " << calib.intrinsics.parameters.param.cy << endl;
	cout << "focal length x: " << calib.intrinsics.parameters.param.fx << endl;
	cout << "focal length y: " << calib.intrinsics.parameters.param.fy << endl;
	cout << "radial distortion coefficients:" << endl;
	cout << "k1: " << calib.intrinsics.parameters.param.k1 << endl;
	cout << "k2: " << calib.intrinsics.parameters.param.k2 << endl;
	cout << "k3: " << calib.intrinsics.parameters.param.k3 << endl;
	cout << "k4: " << calib.intrinsics.parameters.param.k4 << endl;
	cout << "k5: " << calib.intrinsics.parameters.param.k5 << endl;
	cout << "k6: " << calib.intrinsics.parameters.param.k6 << endl;
	cout << "center of distortion in Z=1 plane, x: " << calib.intrinsics.parameters.param.codx << endl;
	cout << "center of distortion in Z=1 plane, y: " << calib.intrinsics.parameters.param.cody << endl;
	cout << "tangential distortion coefficient x: " << calib.intrinsics.parameters.param.p1 << endl;
	cout << "tangential distortion coefficient y: " << calib.intrinsics.parameters.param.p2 << endl;
	cout << "metric radius: " << calib.intrinsics.parameters.param.metric_radius << endl;
	cout << "相机外参：" << endl;
	cout << "rotation：" << endl;
	for (int i = 0; i < 9; i++) {
		cout << calib.extrinsics.rotation[i] << "\t";
		if ((i + 1) % 3 == 0) {
			cout << endl;
		}
	}
	cout << "translation(xyz)：" << endl;
	for (int i = 0; i < 3; i++) {
		cout << calib.extrinsics.translation[i] << "\t";
	}


}