#pragma once
#include <cstdint>
#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <k4a/k4a.h>
#include <k4a/k4atypes.h>
#include <k4arecord/playback.h>
#include <k4arecord/record.h>
#include <k4arecord/types.h>
#include <Python.h>
#include <string>
#include <vector>

using namespace std;

int video2Txt(std::string filename, int start_second);
//void txt2Pcd(std::string txt_dir);
void processTxt(const std::string& txtFile, std::string record_name);
//class Py
//{
//public:
//	Py();
//	void closePy();
//	void pyTxt2Pcd(std::string txt_dir, int start_frame);
//
//private:
//	PyObject* pModule = NULL;
//	PyObject* pFunc = NULL;
//	PyObject* pArgs = NULL;
//};

enum DATA_TYPE
{
	TXT,
	RGBD,
	IMU,
	ALL
};

typedef struct MYIMUSAMPLE {
	int cur_frame;
	k4a_imu_sample_t imu_sample;
}MYIMUSAMPLE;

class KinectRecord {
private:
	k4a_image_t getPointCloudImage(k4a_image_t depth_image, k4a_image_t color_image);
	k4a_image_t getTransColorImage(k4a_image_t depth_image, k4a_image_t color_image, int cur_frame);
	k4a_image_t getTransDepthImage(k4a_image_t depth_image, k4a_image_t color_image, int cur_frame);

	int getData(enum DATA_TYPE type);
	int saveRGBD(k4a_image_t trans_image, int cur_frame);
	int saveTXT(k4a_image_t point_cloud_image, int cur_frame, Eigen::Vector3d v);
	//void pyTxt2Pcd(string txt_dir, int start_frame);
	int saveIMU(vector<MYIMUSAMPLE> my_imu_samples);

public:
	KinectRecord(int length);
	~KinectRecord();
	int initRecord(std::string dirname, std::string filename, int start_second[]);
	int getTXT();
	//int getPCD(Py py, int mode);
	int getPCD(int mode);
	int getRGBD();
	void getIMU();
	void getIMU(int64_t offset_usec, k4a_imu_sample_t* imu_sample);


private:
	const char* path;
	int* start_frame;
	int length;
	std::string filename;
	std::string dirname;

	k4a_playback_t handle;
	k4a_transformation_t trans_handle;
	k4a_record_configuration_t record_config;
	k4a_capture_t capture;
	k4a_calibration_t calibration;	//У׼
	k4a_result_t result = K4A_RESULT_SUCCEEDED;	// result code
};