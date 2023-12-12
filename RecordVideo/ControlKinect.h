#pragma once
#include <k4a/k4a.h>
#include <atomic>

extern std::atomic_bool exiting;

class Kinect
{
public:
	Kinect();
	void setExposureTime(int32_t absoluteExposureValue);
	int startCameras(k4a_device_configuration_t *device_config, bool record_imu);
	int doRecording(const char *recording_filename, int recording_length, k4a_device_configuration_t *device_config);
	void closeDevice();
public:
	k4a_device_t device = NULL;
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	static const int32_t defaultExposureAuto = -12;
	static const int32_t defaultGainAuto = -1;
};