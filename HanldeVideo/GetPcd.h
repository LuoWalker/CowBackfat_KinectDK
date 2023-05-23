#pragma once
#include <string>
#include <k4a/k4a.h>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
using namespace std;

int video2Txt(string filename, int start_second);

enum DATA_TYPE
{
	TXT,
	RGBD,
	ALL
};

class KinectRecord {
private:
	k4a_image_t getPointCloudImage(k4a_image_t depth_image, k4a_image_t color_image);
	k4a_image_t getTransColorImage(k4a_image_t depth_image, k4a_image_t color_image, int cur_frame);

	int getData(enum DATA_TYPE type);
	int saveRGBD(k4a_image_t trans_color_image, int cur_frame);
	int saveTXT(k4a_image_t point_cloud_image, k4a_image_t depth_image, int cur_frame);
	void pyTxt2Pcd(string txt_dir, int start_frame);
public:
	KinectRecord(int length);
	int initRecord(string filename, int start_second[]);
	int getTXT();
	int getPCD(int mode);
	int getRGBD();

private:
	const char* path;
	int* start_frame;
	int length;
	string filename;

	k4a_playback_t handle;
	k4a_transformation_t trans_handle;
	k4a_record_configuration_t record_config;
	k4a_capture_t capture;
	k4a_calibration_t calibration;	//У׼
	k4a_result_t result = K4A_RESULT_SUCCEEDED;	// result code
};

