#include "JudgeCow.h"
#include "ControlKinect.h"

#include <stdio.h>
#include <ctime>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <k4a/k4a.h>
#include <k4arecord/record.h>

using namespace std::chrono;

// call k4a_device_close on every failed CHECK
#define CHECK(x, device) {                                                                                             \
        auto retval = (x);                                                                                             \
        if (retval)                                                                                                    \
        {                                                                                                              \
            std::cerr << "Runtime error: " << #x << " returned " << retval << std::endl;                               \
            k4a_device_close(device);                                                                                  \
            return 1;                                                                                                  \
        }                                                                                                              \
    }

std::atomic_bool exiting(false);

inline static uint32_t k4a_convert_fps_to_uint(k4a_fps_t fps)
{
	uint32_t fps_int;
	switch (fps)
	{
	case K4A_FRAMES_PER_SECOND_5:
		fps_int = 5;
		break;
	case K4A_FRAMES_PER_SECOND_15:
		fps_int = 15;
		break;
	case K4A_FRAMES_PER_SECOND_30:
		fps_int = 30;
		break;
	default:
		fps_int = 0;
		break;
	}
	return fps_int;
}

Kinect::Kinect() {}

void Kinect::setExposureTime(int32_t absoluteExposureValue) {
	if (absoluteExposureValue != defaultExposureAuto) {
		if (K4A_FAILED(k4a_device_set_color_control(device,
			K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
			K4A_COLOR_CONTROL_MODE_MANUAL,
			absoluteExposureValue)))
		{
			std::cerr << "Runtime error: k4a_device_set_color_control() for manual exposure failed " << std::endl;
		}
	}
	else {
		if (K4A_FAILED(k4a_device_set_color_control(device,
			K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
			K4A_COLOR_CONTROL_MODE_AUTO,
			0))) {
			std::cerr << "Runtime error: k4a_device_set_color_control() for auto exposure failed " << std::endl;
		}
	}
}

int Kinect::startCameras(k4a_device_configuration_t *device_config, bool record_imu) {
	// 打开设备
	if (K4A_FAILED(k4a_device_open(0, &device)))
	{
		std::cerr << "Runtime error: k4a_device_open() failed " << std::endl;
	}
	std::cout << "Device open：" << "\n";

	// 打印序列号
	char serial_number_buffer[256];
	size_t serial_number_buffer_size = sizeof(serial_number_buffer);
	CHECK(k4a_device_get_serialnum(device, serial_number_buffer, &serial_number_buffer_size), device);
	std::cout << "Device serial number: " << serial_number_buffer << std::endl;

	// 打印版本信息
	k4a_hardware_version_t version_info;
	CHECK(k4a_device_get_version(device, &version_info), device);
	std::cout << "Device version: " << (version_info.firmware_build == K4A_FIRMWARE_BUILD_RELEASE ? "Rel" : "Dbg")
		<< "; C: " << version_info.rgb.major << "." << version_info.rgb.minor << "." << version_info.rgb.iteration
		<< "; D: " << version_info.depth.major << "." << version_info.depth.minor << "."
		<< version_info.depth.iteration << "[" << version_info.depth_sensor.major << "."
		<< version_info.depth_sensor.minor << "]"
		<< "; A: " << version_info.audio.major << "." << version_info.audio.minor << "."
		<< version_info.audio.iteration << std::endl;

	// 手动曝光或自动曝光
	setExposureTime(this->defaultExposureAuto);

	// 开启摄像头
	CHECK(k4a_device_start_cameras(device, device_config), device);
	if (record_imu) {
		CHECK(k4a_device_start_imu(device), device);
	}
	std::cout << "Cameras started：" << std::endl;
	return 1;
}

int Kinect::doRecording(const char *recording_filename, int recording_length, k4a_device_configuration_t *device_config)
{
	seconds recording_length_seconds(recording_length);
	uint32_t camera_fps = k4a_convert_fps_to_uint(device_config->camera_fps);

	k4a_record_t recording;
	if (K4A_FAILED(k4a_record_create(recording_filename, device, *device_config, &recording)))
	{
		std::cerr << "Unable to create recording file: " << recording_filename << std::endl;
		return 1;
	}
	CHECK(k4a_record_write_header(recording), device);

	k4a_wait_result_t result = K4A_WAIT_RESULT_SUCCEEDED;
	k4a_capture_t capture;
	std::cout << "Started recording：" << std::endl;
	if (recording_length <= 0)
	{
		std::cout << "Press Ctrl-C to stop recording." << std::endl;
	}

	steady_clock::time_point recording_start = steady_clock::now();
	int32_t timeout_ms = 1000 / camera_fps;
	do
	{
		result = k4a_device_get_capture(device, &capture, timeout_ms);
		if (result == K4A_WAIT_RESULT_TIMEOUT)
		{
			continue;
		}
		else if (result != K4A_WAIT_RESULT_SUCCEEDED)
		{
			std::cerr << "Runtime error: k4a_device_get_capture() returned " << result << std::endl;
			break;
		}
		if (isHaveCow(capture) == true) {
			CHECK(k4a_record_write_capture(recording, capture), device);
		}
		k4a_capture_release(capture);
	} while (!exiting && result != K4A_WAIT_RESULT_FAILED &&
		(recording_length < 0 || (steady_clock::now() - recording_start < recording_length_seconds)));

	if (!exiting)
	{
		exiting = true;
		std::cout << "Stopping recording..." << std::endl;
	}


	std::cout << "Saving recording..." << std::endl;
	CHECK(k4a_record_flush(recording), device);
	k4a_record_close(recording);
	exiting = false;
	std::cout << "Done" << std::endl;

	return 0;
}

void Kinect::closeDevice() {
	k4a_device_stop_cameras(device);
	k4a_device_close(device);
}