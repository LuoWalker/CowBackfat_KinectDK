#include "ControlKinect.h"
#include <k4a/k4a.h>

int main(int argc, char *argv[])
{
	Kinect kinect;
	/*if (K4A_RESULT_SUCCEEDED == kinect.isOpen())*/ {
		// Configure a stream of 4096x3072 BRGA color data at 15 frames per second
		k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		config.camera_fps = K4A_FRAMES_PER_SECOND_30;
		config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
		config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
		config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		kinect.startRecord(config);
	}
	return 0;
}