#define _CRT_SECURE_NO_WARNINGS
#define LIMIT_FRAME 30
#include "GetPcd.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <k4a/k4a.hpp>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <direct.h>
#include <Windows.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <algorithm>

using namespace std;
using namespace pcl;
using std::filesystem::directory_iterator;
using std::filesystem::path;


typedef struct VERTEX_3D
{
	int16_t x;
	int16_t y;
	int16_t z;
} VERTEX3D;
typedef struct VERTEX_RGB
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
} VERTEXRGB;

/* 直接获取PCD，速度慢 */
//int KinectRecord::getPCDDirect(k4a_image_t point_cloud_image, k4a_image_t depth_image, int cur_frame) {
//	if (point_cloud_image == NULL || depth_image == NULL) {
//		cout << "image is null" << endl;
//		return -1;
//	}
//	int width = k4a_image_get_width_pixels(point_cloud_image);
//	int height = k4a_image_get_height_pixels(point_cloud_image);
//
//	int16_t* point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);//访问深度图像缓存区
//	uint8_t* depth_image_data = k4a_image_get_buffer(depth_image);
//
//	cout << width << ' ' << height << endl;
//	int count = width * height;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
//
//	point_cloud->is_dense = true;
//	//point_cloud->resize(point_cloud->width*point_cloud->height);
//	//cout << point_cloud->points.size() << endl;
//
//	int pos = filename.find(".");
//	string record_name = filename.substr(0, pos);
//	string pcd_name = record_name + '-' + to_string(cur_frame);
//	cout << pcd_name << endl;
//	int x, y, z, order;
//
//	pcl::PointXYZ point;
//	for (int row = 0; row < height; row++) {
//		for (int col = 0; col < width; col++) {
//			order = row * width + col;
//			x = point_cloud_image_data[3 * order + 0];
//			y = -point_cloud_image_data[3 * order + 1];
//			z = -point_cloud_image_data[3 * order + 2];
//
//			if (x == 0 && y == 0 && z == 0 || z < -1500) {
//				continue;
//			}
//
//			point.x = x; point.y = y; point.z = z + 2400; // 加相机高度
//
//			point_cloud->points.push_back(point);
//		}
//	}
//	cout << point_cloud->points.size() << endl;
//
//	point_cloud->width = point_cloud->points.size();
//	point_cloud->height = 1;
//	string pcd_path = "../PCD/test/" + record_name + '/' + pcd_name + ".pcd";
//
//	pcl::io::savePCDFile(pcd_path, *point_cloud);
//
//	return 1;
//}

int video2Txt(string filename, int start_second) {
	string temp = "../Video/0513/" + filename;
	const char* path = temp.c_str();	//输入的文件路径
	int no_frame = 0;				//帧序号，从0开始
	int start_frame = start_second * 30;	//开始帧，用于截取片段
	cout << path << endl;

	//对文件捕获必须的变量
	k4a_playback_t handle;
	k4a_record_configuration_t record_config;
	k4a_capture_t capture;
	k4a_calibration_t calibration;	//校准
	k4a_result_t result = K4A_RESULT_SUCCEEDED;	// result code

	//打开文件
	result = k4a_playback_open(path, &handle);
	if (result != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to open file: %s\n", path);
		k4a_playback_close(handle);
		handle = NULL;

		return 1;
	}

	//获取文件配置
	result = k4a_playback_set_color_conversion(handle, K4A_IMAGE_FORMAT_COLOR_BGRA32);
	if (result != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to get record configuration for file: device%s\n", path);
		k4a_playback_close(handle);
		handle = NULL;

		return 1;
	}

	result = k4a_playback_get_record_configuration(handle, &record_config);
	if (result != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to get record configuration for file: device%s\n", path);
		k4a_playback_close(handle);
		handle = NULL;

		return 1;
	}

	//取得第一个捕获
	k4a_stream_result_t stream_result = k4a_playback_get_next_capture(handle, &capture);
	if (stream_result == K4A_STREAM_RESULT_EOF)
	{
		printf("ERROR: Recording file is empty %s\n", path);
		result = K4A_RESULT_FAILED;
		goto Exit;
	}
	else if (stream_result == K4A_STREAM_RESULT_FAILED)
	{
		printf("ERROR: Failed to read first capture from file: %s\n", path);
		result = K4A_RESULT_FAILED;
		goto Exit;
	}
	if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(handle, &calibration))
	{
		printf("Failed to get calibration\n");
		goto Exit;
	}

	//第一个捕获获取成功
	if (result == K4A_RESULT_SUCCEEDED)
	{
		k4a_image_t depthImage = k4a_capture_get_depth_image(capture);
		k4a_image_t src_colorImage;
		k4a_image_t transformed_color_image = NULL;
		k4a_image_t dest_color_image = NULL;
		int depth_image_width_pixels = k4a_image_get_width_pixels(depthImage);
		int	depth_image_height_pixels = k4a_image_get_height_pixels(depthImage);
		k4a_transformation_t transformation = NULL;		//转换
		transformation = k4a_transformation_create(&calibration);


		//设置正确的转换视点后的彩色图像格式：彩色类型为BRGA32，图像宽高为深度图像宽高
		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
			depth_image_width_pixels,
			depth_image_height_pixels,
			depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
			&dest_color_image))
		{
			printf("Failed to create destination color image\n");
			goto Exit;
		}

		//进行处理并不断获取新的捕获，处理结束后循环自动停止
		while (true)
		{
			int action = 0;
			//前几帧无彩色图像（捕获的彩色图像为空指针），故忽略前几帧
			if (no_frame >= start_frame) {
				vector<VERTEX3D> g_vet;			//存储三维坐标
				vector<VERTEXRGB> g_vet_color;	//存储相应RGB信息
				VERTEXRGB v_rgb;				//存储颜色信息
				no_frame++;

				depthImage = k4a_capture_get_depth_image(capture);
				src_colorImage = k4a_capture_get_color_image(capture);

				if (depthImage == 0 || src_colorImage == 0) {
					printf(" Fail to get correct image\n");
					goto Exit;
				}

				//判断彩色图像格式是否正确 BGRA32
				k4a_image_format_t format;
				format = k4a_image_get_format(src_colorImage);
				if (format != K4A_IMAGE_FORMAT_COLOR_BGRA32)
				{
					printf(" Fail to convert into color image format BGRA32\n");
					goto Exit;
				}

				//将彩色图从彩色相机的视点转换为深度相机的视点（使他们的像素点对应）
				if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation,
					depthImage,
					src_colorImage,
					dest_color_image))
				{
					printf("Failed to compute transformed color image\n");
					goto Exit;
				}

				cout << "处理第 " << no_frame << " 帧图像" << endl;

				////输出彩色原图
				//cv::Mat rgbframe = cv::Mat(k4a_image_get_height_pixels(src_colorImage),
				//	k4a_image_get_width_pixels(src_colorImage),
				//	CV_8UC4,
				//	k4a_image_get_buffer(src_colorImage));
				//cv::Mat cv_rgbImage_8U;
				//rgbframe.convertTo(cv_rgbImage_8U, CV_8U, 1);
				//string filename_rgb = "RGB_img\\" + to_string(no_frame) + ".png";
				//imwrite(filename_rgb, cv_rgbImage_8U);


				//if (0 != _access(outpath_txt.c_str(), 0)) {
				//	_mkdir(outpath_txt.c_str());
				//}
				//string outfile_txt = outpath_txt + to_string(no_frame) + ".txt";
				//fp = fopen(outfile_txt.c_str(), "w");//在项目目录下输出文件名

				//坐标转换所需变量
				k4a_float2_t p;//二维像素坐标作为输入
				k4a_float3_t ray;//三维世界坐标作为输出
				int valid;//输出是否有效的标记，值为1表明转化结果有效，为0无效
				int i = 0;
				int count_depth_near = 0;//统计深度较小的点的个数（说明有物体）
				//开始对每个像素坐标处理
				for (int row = 0; row < k4a_image_get_width_pixels(depthImage); row++)
				{
					for (int col = 0; col < k4a_image_get_height_pixels(depthImage); col++)
					{
						const short* buffer = reinterpret_cast<const short*>(k4a_image_get_buffer(depthImage));			//访问深度图像缓存区
						uint8_t* color_buffer = reinterpret_cast<uint8_t*>(k4a_image_get_buffer(dest_color_image));		//访问彩色图像缓存区
						v_rgb.b = (int)color_buffer[(size_t(col) * size_t(k4a_image_get_width_pixels(dest_color_image)) + size_t(row)) * size_t(4) + size_t(0)];
						v_rgb.g = (int)color_buffer[(size_t(col) * size_t(k4a_image_get_width_pixels(dest_color_image)) + size_t(row)) * size_t(4) + size_t(1)];
						v_rgb.r = (int)color_buffer[(size_t(col) * size_t(k4a_image_get_width_pixels(dest_color_image)) + size_t(row)) * size_t(4) + size_t(2)];

						short pixelValue = buffer[size_t(col) * size_t(k4a_image_get_width_pixels(depthImage))
							+ size_t(row)];		//计算深度值
						if (pixelValue > 0 && pixelValue < 5000) {		//初步筛选有效深度
							//像素坐标
							p.xy.x = (float)row;
							p.xy.y = (float)col;
							//坐标转换函数（校准类型，要转换的二维像素点坐标，此点深度值，输出相机类型，输出相机类型，输出三维点坐标，有效检测标记）
							if (K4A_RESULT_FAILED == k4a_calibration_2d_to_3d(&calibration, &p,
								(float)pixelValue,
								K4A_CALIBRATION_TYPE_DEPTH,
								K4A_CALIBRATION_TYPE_DEPTH,
								&ray, &valid))
							{
								printf("ERROR: calibration contained invalid transformation parameters. \n");
								goto Exit;
							}
							//有效则输入文件
							VERTEX3D t;
							if (valid) {
								//fprintf(fp, "%d %d %d\n", -(int)ray.xyz.x, -(int)ray.xyz.y, (int)ray.xyz.z);
								t.x = -(int)ray.xyz.x; t.y = -(int)ray.xyz.y; t.z = -(int)ray.xyz.z;
								g_vet.push_back(t);
								g_vet_color.push_back(v_rgb);
								i++;
							}
							//统计深度较小的点的个数（说明有物体）
							//if (pixelValue < 1500) {
							//	count_depth_near++;
							//}
						}
					}
				}

				//FILE* fp_count = NULL;
				//string outpath_count = "./count.txt";
				//fp_count = fopen(outpath_count.c_str(), "a");
				//fprintf(fp_count, "%d:%d\n", no_frame, count_depth_near);
				//fclose(fp_count);

				FILE* fp = NULL;
				string outpath_txt = "PointCloudData\\" + filename + "\\";

				//if (count_depth_near > 13000) {
				if (TRUE) {
					//输出深度图视角的彩色图
					cv::Mat rgbdframe = cv::Mat(k4a_image_get_height_pixels(dest_color_image),
						k4a_image_get_width_pixels(dest_color_image),
						CV_8UC4, k4a_image_get_buffer(dest_color_image));
					cv::Mat cv_rgbdImage_8U;
					rgbdframe.convertTo(cv_rgbdImage_8U, CV_8U, 1);
					string outpath_rgbd = "RGBD_img\\" + filename + "\\";
					if (0 != _access(outpath_rgbd.c_str(), 0)) {
						_mkdir(outpath_rgbd.c_str());
					}
					string outfile_rgbd = outpath_rgbd + to_string(no_frame) + ".png";
					imwrite(outfile_rgbd, cv_rgbdImage_8U);

					//输出txt点云
					if (0 != _access(outpath_txt.c_str(), 0)) {
						_mkdir(outpath_txt.c_str());
					}
					string outfile_txt = outpath_txt + to_string(no_frame) + ".txt";
					fp = fopen(outfile_txt.c_str(), "w");//在项目目录下输出文件名
					//fprintf(fp, "%d\n", i); // 点云总数
					for (int j = 0; j < i; j++)
					{
						//fprintf(fp, "%d %d %d\n", g_vet[j].x, g_vet[j].y, g_vet[j].z);
						fprintf(fp, "%d %d %d %f %f %f\n", g_vet[j].x, g_vet[j].y, g_vet[j].z, ((float)g_vet_color[j].r / 255), ((float)g_vet_color[j].g / 255), ((float)g_vet_color[j].b / 255));
					}
					fclose(fp);
				}
				//此时已可以利用深度图进行处理，但opencv仅支持显示8位灰度图，若要可视化，则需进一步转化
			}
			k4a_capture_release(capture);

			// Get a next capture
			switch (k4a_playback_get_next_capture(handle, &capture))
			{
			case K4A_WAIT_RESULT_SUCCEEDED:
				no_frame++;

				break;
			case K4A_WAIT_RESULT_FAILED:
				printf("ERROR: Failed to read next capture from file: %s\n", path);
				goto Exit;
			case K4A_STREAM_RESULT_EOF:
				printf("%d\n", no_frame);
				printf("FINISH!\n");
				goto Exit;

			}
		}
	}


Exit:
	k4a_playback_close(handle);
	handle = NULL;

}

KinectRecord::KinectRecord(int length) {
	handle = NULL;
	trans_handle = NULL;
	capture = NULL;
	this->length = length;
	start_frame = new int[length];
}

int KinectRecord::initRecord(string filename, int start_second[]) {
	this->filename = filename;
	string inPath = "E:/luowenkuo/Video/0111/" + filename + ".mkv";
	//string temp = "../RecordVideo/" + filename;
	const char* path = inPath.c_str();	//输入的文件路径

	string outPath = "../PCD/origin/0111/" + filename;
	const char* destPath = outPath.c_str();	//输出的文件路径

	if (_access(destPath, 0) == 0) { //判断该文件夹是否存在
		cout << "已跳过:" << filename << endl;
		return 0;
	}

	for (int i = 0; i < length; i++)
	{
		start_frame[i] = start_second[i] * 30;	//开始帧，用于截取片段
	}

	//打开文件
	result = k4a_playback_open(path, &handle);
	if (result != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to open file: %s\n", path);
		k4a_playback_close(handle);
		handle = NULL;

		return 0;
	}

	//获取文件配置
	result = k4a_playback_get_record_configuration(handle, &record_config);
	k4a_playback_set_color_conversion(handle, K4A_IMAGE_FORMAT_COLOR_BGRA32);
	if (result != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to get record configuration for file: device%s\n", path);
		k4a_playback_close(handle);
		handle = NULL;

		return 0;
	}
	//获取校准数据
	if (k4a_playback_get_calibration(handle, &calibration)) {
		printf("Failed to get calibration\n");
	}
	//获取准换句柄
	trans_handle = k4a_transformation_create(&calibration);
	return 1;
}

k4a_image_t KinectRecord::getPointCloudImage(k4a_image_t depth_image, k4a_image_t color_image) {

	//int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
	//int color_image_height_pixels = k4a_image_get_height_pixels(color_image);

	int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
	int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);

	// 创建点云图片
	k4a_image_t point_cloud_image = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		depth_image_width_pixels,
		depth_image_height_pixels,
		depth_image_width_pixels * 3 * (int)sizeof(int16_t),
		&point_cloud_image))
	{
		cout << "Failed to create point cloud image" << endl;
		return NULL;
	}

	// 初始化点云图片
	if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(trans_handle,
		depth_image,
		K4A_CALIBRATION_TYPE_DEPTH,
		point_cloud_image))
	{
		cout << "Failed to compute point cloud" << endl;
		return NULL;
	}
	k4a_image_release(depth_image);
	return point_cloud_image;
}

k4a_image_t KinectRecord::getTransColorImage(k4a_image_t depth_image, k4a_image_t color_image, int cur_frame) {

	int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
	int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);

	//创建目标图
	k4a_image_t trans_color_image = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
		depth_image_width_pixels,
		depth_image_height_pixels,
		depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
		&trans_color_image))
	{
		cout << "Failed to create transformed color image" << endl;
		return NULL;
	}

	//将彩色图图转为深度图视点
	if (K4A_RESULT_SUCCEEDED !=
		k4a_transformation_color_image_to_depth_camera(trans_handle, depth_image, color_image, trans_color_image))
	{
		cout << "Failed to compute transformed color image" << endl;
		return NULL;
	}
	else {
		return trans_color_image;
	}
}

k4a_image_t KinectRecord::getTransDepthImage(k4a_image_t depth_image, k4a_image_t color_image, int cur_frame) {
	int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
	int color_image_height_pixels = k4a_image_get_height_pixels(color_image);

	//创建目标图
	k4a_image_t trans_depth_image = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
		color_image_width_pixels,
		color_image_height_pixels,
		color_image_width_pixels * (int)sizeof(uint16_t),
		&trans_depth_image))
	{
		cout << "Failed to create transformed color image" << endl;
		return NULL;
	}

	//将彩色图图转为深度图视点
	if (K4A_RESULT_SUCCEEDED !=
		k4a_transformation_depth_image_to_color_camera(trans_handle, depth_image, trans_depth_image))
	{
		cout << "Failed to compute transformed color image" << endl;
		return NULL;
	}
	else {
		return trans_depth_image;
	}
}

int KinectRecord::getData(enum DATA_TYPE type) {
	int cur_frame = 0;
	for (cur_frame; cur_frame < 6; cur_frame++) {
		//前六帧无彩色图，跳过
		k4a_playback_get_next_capture(handle, &capture);
	}
	cout << "开始处理：" << filename << "\n";
	//Py_SetPythonHome(L"D:\\anaconda3\\envs\\BCS"); // 定义python解释器
	//Py_Initialize(); // 初始化python接口
	std::vector<std::thread> threads;
	for (int i = 0; i < length; i++)
	{
		// 跳过无用帧
		while (cur_frame <= start_frame[i]) {
			k4a_playback_get_next_capture(handle, &capture);
			k4a_capture_release(capture);
			cur_frame++;
		}

		int n = 0;

		// 开始输出
		while (cur_frame <= start_frame[i] + LIMIT_FRAME) {
			k4a_stream_result_t stream_result = k4a_playback_get_next_capture(handle, &capture);

			if (stream_result == K4A_STREAM_RESULT_EOF)
			{
				printf("ERROR: Recording file is empty: %s\n", path);
				result = K4A_RESULT_FAILED;
				return -1;
			}
			else if (stream_result == K4A_STREAM_RESULT_FAILED)
			{
				printf("ERROR: Failed to read first capture from file: %s\n", path);
				result = K4A_RESULT_FAILED;
				return -1;
			}

			k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
			k4a_image_t color_image = k4a_capture_get_color_image(capture);

			//if (depth_image == NULL)
			//{
			//	cout << "Failed to get depth image from capture" << endl;
			//	return -1;
			//}
			//if (color_image == NULL)
			//{
			//	cout << "Failed to get color image from capture" << endl;
			//	return -1;
			//}
			k4a_image_t trans_color_image = NULL;
			//k4a_image_t trans_depth_image = NULL;
			trans_color_image = getTransColorImage(depth_image, color_image, cur_frame);
			//trans_depth_image = getTransDepthImage(depth_image, color_image, cur_frame);

			if (type == TXT || type == ALL) {
				k4a_image_t point_cloud_image = NULL;
				point_cloud_image = getPointCloudImage(depth_image, color_image);
				n++;
				threads.emplace_back(&KinectRecord::saveTXT, this, point_cloud_image, depth_image, trans_color_image, cur_frame);
			}
			if (type == RGBD || type == ALL) {
				saveRGBD(trans_color_image, cur_frame);
			}

			k4a_capture_release(capture);
			if (n % 6 == 0) {
				//等待所有线程执行完毕
				for (auto& thread : threads) {
					thread.join();
				}
				threads.clear();	
			}
			cur_frame++;
		}
		//pyTxt2Pcd(filename, start_frame[i]);
		for (auto& thread : threads) {
			thread.join();
		}
		threads.clear();
		cout << start_frame[i] / 30 << ' ';
	}

	//Py_Finalize(); //结束python接口
	cout << "处理完成：" << filename << "\n";
	return 1;

}

int KinectRecord::saveRGBD(k4a_image_t trans_image, int cur_frame) {
	int width_pixels = k4a_image_get_width_pixels(trans_image);
	int height_pixels = k4a_image_get_height_pixels(trans_image);
	//输出深度图视角的RGBD
	cv::Mat rgbdframe = cv::Mat(height_pixels, width_pixels, CV_8UC4, k4a_image_get_buffer(trans_image));
	cv::Mat cv_rgbdImage_8U;
	rgbdframe.convertTo(cv_rgbdImage_8U, CV_8U, 1);

	//输出彩色图视角的RGBD
	//cv::Mat rgbdframe = cv::Mat(height_pixels, width_pixels, CV_8UC4, (uint8_t *)k4a_image_get_buffer(trans_image));
	//cv::Mat cv_rgbdImage_8U;
	//rgbdframe.convertTo(cv_rgbdImage_8U, CV_8U, 1);

	string outpath_rgbd = "./RGBD_img/" + filename + "/";
	if (0 != _access(outpath_rgbd.c_str(), 0)) {
		_mkdir(outpath_rgbd.c_str());
	}
	string png_name = to_string(cur_frame) + ".png";
	string outfile_rgbd = outpath_rgbd + png_name;
	imwrite(outfile_rgbd, cv_rgbdImage_8U);
	//cout << png_name << endl;
	return 1;
}

int KinectRecord::saveTXT(k4a_image_t point_cloud_image, k4a_image_t depth_image, k4a_image_t color_image, int cur_frame) {
	if (point_cloud_image == NULL || depth_image == NULL) {
		cout << "image is null" << endl;
		return -1;
	}
	int width = k4a_image_get_width_pixels(point_cloud_image);
	int height = k4a_image_get_height_pixels(color_image);

	int16_t* point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);//访问深度图像缓存区
	//uint8_t* depth_image_data = k4a_image_get_buffer(depth_image);
	uint8_t* color_buffer = k4a_image_get_buffer(color_image);		//访问彩色图像缓存区

	int count = width * height;

	int pos = filename.find(".");
	string record_name = filename.substr(0, pos);
	string txt_name = filename + "-" + to_string(cur_frame) + ".pcd";

	//string txt_dir = "./PointCloudData/0111/" + filename + "/";
	string txt_dir = "../PCD/origin/0111/" + filename + "/";
	if (0 != _access(txt_dir.c_str(), 0)) {
		_mkdir(txt_dir.c_str());
	}

	//FILE* fp = NULL;
	string txt_path = txt_dir + txt_name;
	//fp = fopen(txt_path.c_str(), "w");//在项目目录下输出文件名
	//cout << txt_name << endl;

	int order;
	pcl::PointXYZRGB point;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			order = row * width + col;
			point.x = point_cloud_image_data[3 * order + 0];
			point.y = -point_cloud_image_data[3 * order + 1];
			point.z = -point_cloud_image_data[3 * order + 2];


			point.b = color_buffer[4 * order + 0];
			point.g = color_buffer[4 * order + 1];
			point.r = color_buffer[4 * order + 2];

			if (point.x == 0 && point.y == 0 && point.z == 0 || point.z < -1500) {
				continue;
			}
			cloud->push_back(point);

			point.z += 1000; // 加相机高度
			//fprintf(fp, "%d %d %d %d %d %d\n", point.x, point.y, point.z, rgb.r, rgb.g, rgb.b);
		}
	}
	pcl::io::savePCDFileASCII(txt_path, *cloud);
	cloud.reset();
	return 1;
}

Py::Py() {
	Py_SetPythonHome(L"D:\\environments\\miniconda3\\envs\\BF"); // 定义python解释器
	Py_Initialize(); // 初始化python接口

	string command = "conda activate BF";
	system(command.c_str()); // 激活conda环境

	PyRun_SimpleString("import sys");
	PyRun_SimpleString("import os");
	PyRun_SimpleString("sys.path.append('Script')"); // 定义路径
	PyRun_SimpleString("print(os.getcwd())");

	if (pModule = PyImport_ImportModule("get_pcd")) {
		if (pFunc = PyObject_GetAttrString(pModule, "txt_pcd")) {
		}
		else {
			cout << "导入失败" << endl;
		}
	}
	else {
		cout << "文件失败" << endl;
		PyErr_Print();
	}
}
void Py::closePy() {
	Py_Finalize(); //结束python接口
}

void Py::pyTxt2Pcd(string txt_dir, int start_frame) {
	PyObject* result;
	const char* txt_dirc = txt_dir.c_str();
	cout << txt_dirc << endl;

	pArgs = PyTuple_New(2);
	PyTuple_SetItem(pArgs, 0, Py_BuildValue("s", txt_dirc));
	PyTuple_SetItem(pArgs, 1, Py_BuildValue("i", start_frame));

	result = PyObject_CallObject(pFunc, pArgs); // 调用函数
}

void txt2Pcd(string record_name) {
	string txt_path = "./PointCloudData/" + record_name + "/";
	std::vector<std::string> txtFiles;

	for (auto& v : directory_iterator(txt_path))
	{
		txtFiles.push_back(v.path().string());
	}
	// 启动多线程处理每个 txt 文件
	std::vector<std::thread> threads;
	int n_threads = 20;
	cout << "开始处理：" << record_name << "\n";
	int n_files = 0;
	for (const auto& txtFile : txtFiles) {
		n_files++;

		threads.emplace_back([txtFile, record_name]() {
			processTxt(txtFile, record_name);
			});

		if (n_files % n_threads == 0)
		{
			Sleep(5000);
		}
	}
	// 等待所有线程执行完毕
	for (auto& thread : threads) {
		thread.join();
	}

	cout << "处理完成：" << record_name << "\n";

}
void processTxt(const std::string& txtFile, const std::string record_name) {
	string filename = path(txtFile).filename().string();
	string pcd_path = "..\\PCD\\origin\\0310\\" + record_name + "\\";
	if (0 != _access(pcd_path.c_str(), 0)) {
		_mkdir(pcd_path.c_str());
	}
	std::ifstream file(txtFile);
	std::string line;
	pcl::PointXYZRGB point;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	while (getline(file, line)) {
		std::stringstream ss(line);
		ss >> point.x;
		ss >> point.y;
		ss >> point.z;
		float f;
		std::uint8_t r = 0, g = 0, b = 0;
		ss >> f; r = f;
		ss >> f; g = f;
		ss >> f; b = f;
		point.r = r;
		point.g = g;
		point.b = b;
		cloud->push_back(point);
	}
	file.close();

	string pcd_file = pcd_path + record_name + "-" + filename.substr(0, filename.rfind(".")) + ".pcd";
	pcl::io::savePCDFileASCII(pcd_file, *cloud);
}

int KinectRecord::getTXT() {
	enum DATA_TYPE type = TXT;
	getData(type);
	return 1;
}

int KinectRecord::getPCD(int mode) {
	if (mode == 0) {
		enum DATA_TYPE type = ALL;
		getData(type);
		//py.pyTxt2Pcd(filename, start_frame[0]);
	}
	else if (mode == 1) {
		getTXT();
		//py.pyTxt2Pcd(filename, start_frame[0]);
	}
	else if (mode == 2) {
		//py.pyTxt2Pcd(filename, start_frame[0]);
		txt2Pcd(filename);
	}
	return 1;
}

int KinectRecord::getRGBD() {
	enum DATA_TYPE type = RGBD;
	getData(type);
	return 1;
}
