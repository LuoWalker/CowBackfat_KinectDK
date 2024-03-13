#include "ControlKinect.h"
#include "JudgeCow.h"
#include <k4a/k4a.h>

int main(int argc, char *argv[])
{
	Kinect kinect;
	k4a_device_configuration_t config = { K4A_IMAGE_FORMAT_COLOR_MJPG, K4A_COLOR_RESOLUTION_1080P, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_FRAMES_PER_SECOND_30,
											false, // synchronized_images_only
											0, // depth_delay_off_color_usec
											K4A_WIRED_SYNC_MODE_STANDALONE, // wired_sync_mode
											//0, // subordinate_delay_off_master_usec
											false // disable_streaming_indicator
	};
	//kinect.startCameras(&config, true);
	//kinect.doRecording("./output.mkv", 3, &config);
	//kinect.doRecording("./output1.mkv", 3, &config);
	isHaveCow(kinect.device);
	//kinect.closeDevice();

	return 0;
}