#include "JudgeCow.h"
#include <iostream>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/bgsegm.hpp>
#include <opencv2/viz.hpp>
using namespace cv;
using namespace std;

//void moveObjDetect() {
//	Ptr<BackgroundSubtractor> pBackSub = bgsegm::createBackgroundSubtractorCNT();
//	VideoCapture capture("F:/luowenkuo/Video/0513/05131.mkv");
//
//	Mat frame, fgMask;
//	while (true) {
//		capture >> frame;
//		if (frame.empty())
//			break;
//		//update the background model
//		pBackSub->apply(frame, fgMask, -1);
//		//get the frame number and write it on the current frame
//		rectangle(frame, cv::Point(10, 2), cv::Point(100, 20),
//			cv::Scalar(255, 255, 255), -1);
//		stringstream ss;
//		ss << capture.get(CAP_PROP_POS_FRAMES);
//		string frameNumberString = ss.str();
//		putText(frame, frameNumberString.c_str(), cv::Point(15, 15),
//			FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
//		//show the current frame and the fg masks
//		imshow("Frame", frame);
//		imshow("FG Mask", fgMask);
//		//get the input from the keyboard
//		int keyboard = waitKey(30);
//		if (keyboard == 'q' || keyboard == 27)
//			break;
//	}
//}

bool isHaveCow(k4a_device_t device) {
	k4a_result_t result;
	k4a_capture_t capture;
	k4a_playback_t handle;
	k4a_record_configuration_t record_config;
	const char* path = "F:\\luowenkuo\\Video\\0513\\05131.mkv";
	//打开文件
	result = k4a_playback_open(path, &handle);

	//获取文件配置
	result = k4a_playback_set_color_conversion(handle, K4A_IMAGE_FORMAT_COLOR_BGRA32);

	result = k4a_playback_get_record_configuration(handle, &record_config);

	//取得第一个捕获
	k4a_stream_result_t stream_result = k4a_playback_get_next_capture(handle, &capture);
	k4a_image_t curDepthImage;
	Mat curDepthMat, thresholded_image;
	uint8_t threshold_value;

	while (true) {
		//第一个捕获获取成功
		if (stream_result == K4A_RESULT_SUCCEEDED)
		{
			// 读取当前帧
			curDepthImage = k4a_capture_get_depth_image(capture);

			// 获取深度图数据
			curDepthMat = getMatFromk4a(curDepthImage);

			// 设置深度阈值
			threshold_value = 700;

			// 使用阈值化操作
			threshold(curDepthMat, thresholded_image, threshold_value, 255, THRESH_BINARY);

			// 显示阈值化图
			int white_pixel_count = countNonZero(thresholded_image);
			putText(thresholded_image, to_string(white_pixel_count), Point(50, 50), FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1);
			imshow("result", thresholded_image);
		}
		else
		{
			break;
		}
		k4a_capture_release(capture);
		stream_result = k4a_playback_get_next_capture(handle, &capture);
		// 检测按键，按Esc键退出循环
		char key = waitKey(30);
		if (key == 27)
			break;
	}
	// 关闭摄像头
	destroyAllWindows();
	return true;
}

bool isHaveCow(k4a_capture_t capture) {
	k4a_result_t result;

	k4a_image_t curDepthImage;
	Mat curDepthMat, thresholded_image;
	uint8_t threshold_value;


	// 读取当前帧
	curDepthImage = k4a_capture_get_depth_image(capture);

	// 获取深度图数据
	curDepthMat = getMatFromk4a(curDepthImage);

	// 设置深度阈值
	threshold_value = 700;

	// 使用阈值化操作
	threshold(curDepthMat, thresholded_image, threshold_value, 255, THRESH_BINARY);

	// 显示阈值化图
	int white_pixel_count = countNonZero(thresholded_image);
	putText(thresholded_image, to_string(white_pixel_count), Point(50, 50), FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1);
	imshow("result", thresholded_image);

	return true;
}
Mat getMatFromk4a(k4a_image_t k4aImage) {
	// 获取图像属性
	int width = k4a_image_get_width_pixels(k4aImage);
	int height = k4a_image_get_height_pixels(k4aImage);
	int stride = k4a_image_get_stride_bytes(k4aImage);

	// 获取图像数据指针
	uint8_t* k4aData = k4a_image_get_buffer(k4aImage);

	// 创建对应 OpenCV Mat
	Mat cvImage(height, width, CV_16U, k4aData);
	cvImage.convertTo(cvImage, CV_8U, 1);

	// 如果需要 BGR 格式而不是灰度图，请进行相应的颜色通道转换
	//cv::cvtColor(cvImage, cvImage, cv::COLOR_GRAY2BGR);

	return cvImage;
}