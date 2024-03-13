#pragma once
#include <opencv.hpp>
#include <k4a/k4a.h>
//void moveObjDetect();
bool isHaveCow(k4a_device_t device);
bool isHaveCow(k4a_capture_t capture);
cv::Mat getMatFromk4a(k4a_image_t image, int mode);