#define _CRT_SECURE_NO_WARNINGS
#define PROCESS_FRAME_NUMBER 1000
#pragma comment(lib, "k4a.lib")
#include "GetPcd.h"

typedef struct VERTEX_3D
{
	int x;
	int y;
	int z;
} VERTEX3D;

typedef struct VERTEX_RGB
{
	int r;
	int g;
	int b;
} VERTEXRGB;

using namespace std;

int Video2Txt(const char* path, int start_second) {
	const char* filename = path;	//输入的文件路径
	int no_frame = 0;				//帧序号，从0开始
	int start_frame = start_second * 30;	//开始帧，用于截取片段

	//对文件捕获必须的变量
	k4a_playback_t handle;
	k4a_record_configuration_t record_config;
	k4a_capture_t capture;
	k4a_calibration_t calibration;//校准
	k4a_result_t result = K4A_RESULT_SUCCEEDED;		// result code

	//打开文件
	result = k4a_playback_open(filename, &handle);
	if (result != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to open file: %s\n", filename);
		k4a_playback_close(handle);
		handle = NULL;

		return 1;
	}

	//获取文件配置
	result = k4a_playback_set_color_conversion(handle, K4A_IMAGE_FORMAT_COLOR_BGRA32);
	if (result != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to get record configuration for file: devixe%s\n", filename);
		k4a_playback_close(handle);
		handle = NULL;

		return 1;
	}

	result = k4a_playback_get_record_configuration(handle, &record_config);
	if (result != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to get record configuration for file: device%s\n", filename);
		k4a_playback_close(handle);
		handle = NULL;

		return 1;
	}

	//取得第一个捕获
	k4a_stream_result_t stream_result = k4a_playback_get_next_capture(handle, &capture);
	if (stream_result == K4A_STREAM_RESULT_EOF)
	{
		printf("ERROR: Recording file is empty: %s\n", filename);
		result = K4A_RESULT_FAILED;
		goto Exit;
	}
	else if (stream_result == K4A_STREAM_RESULT_FAILED)
	{
		printf("ERROR: Failed to read first capture from file: %s\n", filename);
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


		int t = 6;
		int cur_frame = no_frame;
		//进行处理并不断获取新的捕获

		while (no_frame <= PROCESS_FRAME_NUMBER)
		{
			t--;
			//前几帧无彩色图像（捕获的彩色图像为空指针），故忽略前几帧
			if (t <= 0 && cur_frame>=start_frame) {
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
				if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation, depthImage, src_colorImage, dest_color_image))
				{
					printf("Failed to compute transformed color image\n");
					goto Exit;
				}

				cout << "处理第 " << no_frame << "帧图像" << endl;

				//输出深度图视角的彩色图
				cv::Mat rgbdframe = cv::Mat(k4a_image_get_height_pixels(dest_color_image), k4a_image_get_width_pixels(dest_color_image), CV_8UC4, k4a_image_get_buffer(dest_color_image));
				cv::Mat cv_rgbdImage_8U;
				rgbdframe.convertTo(cv_rgbdImage_8U, CV_8U, 1);
				string filename_rgbd = "RGBD_img\\" + to_string(no_frame) + ".png";
				imwrite(filename_rgbd, cv_rgbdImage_8U);

				//输出彩色原图
				cv::Mat rgbframe = cv::Mat(k4a_image_get_height_pixels(src_colorImage), k4a_image_get_width_pixels(src_colorImage), CV_8UC4, k4a_image_get_buffer(src_colorImage));
				cv::Mat cv_rgbImage_8U;
				rgbframe.convertTo(cv_rgbImage_8U, CV_8U, 1);
				string filename_rgb = "RGB_img\\" + to_string(no_frame) + ".png";
				imwrite(filename_rgb, cv_rgbImage_8U);


				FILE* fp = NULL;
				string outfile = "PointCloudData\\" + to_string(no_frame) + ".txt";
				fp = fopen(outfile.c_str(), "w");//在项目目录下输出文件名
				//坐标转换所需变量
				k4a_float2_t p;//二维像素坐标作为输入
				k4a_float3_t ray;//三维世界坐标作为输出
				int valid;//输出是否有效的标记，值为1表明转化结果有效，为0无效
				int i = 0;
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

						short pixelValue = buffer[size_t(col) * size_t(k4a_image_get_width_pixels(depthImage)) + size_t(row)];		//计算深度值
						if (pixelValue > 0 && pixelValue < 5000) {		//初步筛选有效深度
							//像素坐标
							p.xy.x = (float)row;
							p.xy.y = (float)col;
							//坐标转换函数（校准类型，要转换的二维像素点坐标，此点深度值，输出相机类型，输出相机类型，输出三维点坐标，有效检测标记）
							if (K4A_RESULT_FAILED == k4a_calibration_2d_to_3d(&calibration, &p, (float)pixelValue, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid)) {
								printf("ERROR: calibration contained invalid transformation parameters. \n");
								goto Exit;
							}
							//有效则输入文件
							VERTEX3D t;
							if (valid) {
								//fprintf(fp, "%d %d %d\n", -(int)ray.xyz.x, -(int)ray.xyz.y, (int)ray.xyz.z);
								t.x = -(int)ray.xyz.x; t.y = -(int)ray.xyz.y; t.z = (int)ray.xyz.z;
								g_vet.push_back(t);
								g_vet_color.push_back(v_rgb);
								i++;
							}
						}
					}
				}
				//fprintf(fp, "%d\n", i);
				for (int j = 0; j < i; j++)
				{
					//fprintf(fp, "%d %d %d\n", g_vet[j].x, g_vet[j].y, g_vet[j].z);
					fprintf(fp, "%d %d %d %f %f %f\n", g_vet[j].x, g_vet[j].y, g_vet[j].z, ((float)g_vet_color[j].r / 255), ((float)g_vet_color[j].g / 255), ((float)g_vet_color[j].b / 255));
				}
				fclose(fp);

				//此时已可以利用深度图进行处理，但opencv仅支持显示8位灰度图，若要可视化，则需进一步转化
			}
			k4a_capture_release(capture);

			// Get a next capture
			switch (k4a_playback_get_next_capture(handle, &capture))
			{
			case K4A_WAIT_RESULT_SUCCEEDED:
				cur_frame++;
				break;
			case K4A_WAIT_RESULT_FAILED:
				printf("ERROR: Failed to read next capture from file: %s\n", filename);
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

void PyTxt2Pcd(string name_txt,string name_pcd) {
	Py_SetPythonHome(L"D:\\anaconda3\\envs\\BCS"); // 定义python解释器
	Py_Initialize(); // 初始化python接口
	string command = "conda activate BCS";
	system(command.c_str()); // 激活conda环境

	PyRun_SimpleString("import sys");
	PyRun_SimpleString("import os");
	PyRun_SimpleString("sys.path.append('./Script')"); // 定义路径
	PyRun_SimpleString("print(os.getcwd())");

	PyObject *pModule, *pFunc, *pArgs;

	if (pModule = PyImport_ImportModule("get_pcd")) {
		if (pFunc = PyObject_GetAttrString(pModule, "txt_pcd")) {
			pArgs = Py_BuildValue("ss", name_txt, name_pcd);

			PyObject_CallObject(pFunc, pArgs); // 调用函数
		}
		else {
			cout << "导入失败" << endl;
		}
	}
	else {
		cout << "文件失败" << endl;
	}
	Py_Finalize(); //结束python接口
}