#include "GetPcd.h"
#include "GetPcd.cpp"

typedef struct VERTEX_3D
{
	int x;
	int y;
	int z;
} VERTEX3D;
typedef struct VERTEX_RGB
{
	float r;
	float g;
	float b;
} VERTEXRGB;

int video2Txt(string filename, int start_second) {
	string temp = "../Video/" + filename;
	const char* path = temp.c_str();	//������ļ�·��
	int no_frame = 0;				//֡��ţ���0��ʼ
	int start_frame = start_second * 30;	//��ʼ֡�����ڽ�ȡƬ��

	//���ļ��������ı���
	k4a_playback_t handle;
	k4a_record_configuration_t record_config;
	k4a_capture_t capture;
	k4a_calibration_t calibration;	//У׼
	k4a_result_t result = K4A_RESULT_SUCCEEDED;	// result code

	//���ļ�
	result = k4a_playback_open(path, &handle);
	if (result != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to open file: %s\n", path);
		k4a_playback_close(handle);
		handle = NULL;

		return 1;
	}

	//��ȡ�ļ�����
	result = k4a_playback_set_color_conversion(handle, K4A_IMAGE_FORMAT_COLOR_BGRA32);
	if (result != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to get record configuration for file: devixe%s\n", path);
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

	//ȡ�õ�һ������
	k4a_stream_result_t stream_result = k4a_playback_get_next_capture(handle, &capture);
	if (stream_result == K4A_STREAM_RESULT_EOF)
	{
		printf("ERROR: Recording file is empty: %s\n", path);
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

	//��һ�������ȡ�ɹ�
	if (result == K4A_RESULT_SUCCEEDED)
	{
		k4a_image_t depthImage = k4a_capture_get_depth_image(capture);
		k4a_image_t src_colorImage;
		k4a_image_t transformed_color_image = NULL;
		k4a_image_t dest_color_image = NULL;
		int depth_image_width_pixels = k4a_image_get_width_pixels(depthImage);
		int	depth_image_height_pixels = k4a_image_get_height_pixels(depthImage);
		k4a_transformation_t transformation = NULL;		//ת��
		transformation = k4a_transformation_create(&calibration);


		//������ȷ��ת���ӵ��Ĳ�ɫͼ���ʽ����ɫ����ΪBRGA32��ͼ�����Ϊ���ͼ�����
		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
			depth_image_width_pixels,
			depth_image_height_pixels,
			depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
			&dest_color_image))
		{
			printf("Failed to create destination color image\n");
			goto Exit;
		}

		int cur_frame = no_frame;

		//���д��������ϻ�ȡ�µĲ��񣬴���������ѭ���Զ�ֹͣ
		while (true)
		{
			int action = 0;
			//ǰ��֡�޲�ɫͼ�񣨲���Ĳ�ɫͼ��Ϊ��ָ�룩���ʺ���ǰ��֡
			if (cur_frame >= start_frame) {
				vector<VERTEX3D> g_vet;			//�洢��ά����
				vector<VERTEXRGB> g_vet_color;	//�洢��ӦRGB��Ϣ
				VERTEXRGB v_rgb;				//�洢��ɫ��Ϣ
				no_frame++;

				depthImage = k4a_capture_get_depth_image(capture);
				src_colorImage = k4a_capture_get_color_image(capture);

				if (depthImage == 0 || src_colorImage == 0) {
					printf(" Fail to get correct image\n");
					goto Exit;
				}

				//�жϲ�ɫͼ���ʽ�Ƿ���ȷ BGRA32
				k4a_image_format_t format;
				format = k4a_image_get_format(src_colorImage);
				if (format != K4A_IMAGE_FORMAT_COLOR_BGRA32)
				{
					printf(" Fail to convert into color image format BGRA32\n");
					goto Exit;
				}

				//����ɫͼ�Ӳ�ɫ������ӵ�ת��Ϊ���������ӵ㣨ʹ���ǵ����ص��Ӧ��
				if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation,
					depthImage,
					src_colorImage,
					dest_color_image))
				{
					printf("Failed to compute transformed color image\n");
					goto Exit;
				}

				cout << "������ " << no_frame << " ֡ͼ��" << endl;

				////�����ɫԭͼ
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
				//fp = fopen(outfile_txt.c_str(), "w");//����ĿĿ¼������ļ���

				//����ת���������
				k4a_float2_t p;//��ά����������Ϊ����
				k4a_float3_t ray;//��ά����������Ϊ���
				int valid;//����Ƿ���Ч�ı�ǣ�ֵΪ1����ת�������Ч��Ϊ0��Ч
				int i = 0;
				int count_depth_near = 0;//ͳ����Ƚ�С�ĵ�ĸ�����˵�������壩
				//��ʼ��ÿ���������괦��
				for (int row = 0; row < k4a_image_get_width_pixels(depthImage); row++)
				{
					for (int col = 0; col < k4a_image_get_height_pixels(depthImage); col++)
					{
						const short* buffer = reinterpret_cast<const short*>(k4a_image_get_buffer(depthImage));			//�������ͼ�񻺴���
						uint8_t* color_buffer = reinterpret_cast<uint8_t*>(k4a_image_get_buffer(dest_color_image));		//���ʲ�ɫͼ�񻺴���
						v_rgb.b = (int)color_buffer[(size_t(col) * size_t(k4a_image_get_width_pixels(dest_color_image)) + size_t(row)) * size_t(4) + size_t(0)];
						v_rgb.g = (int)color_buffer[(size_t(col) * size_t(k4a_image_get_width_pixels(dest_color_image)) + size_t(row)) * size_t(4) + size_t(1)];
						v_rgb.r = (int)color_buffer[(size_t(col) * size_t(k4a_image_get_width_pixels(dest_color_image)) + size_t(row)) * size_t(4) + size_t(2)];

						short pixelValue = buffer[size_t(col) * size_t(k4a_image_get_width_pixels(depthImage))
							+ size_t(row)];		//�������ֵ
						if (pixelValue > 0 && pixelValue < 5000) {		//����ɸѡ��Ч���
							//��������
							p.xy.x = (float)row;
							p.xy.y = (float)col;
							//����ת��������У׼���ͣ�Ҫת���Ķ�ά���ص����꣬�˵����ֵ�����������ͣ����������ͣ������ά�����꣬��Ч����ǣ�
							if (K4A_RESULT_FAILED == k4a_calibration_2d_to_3d(&calibration, &p,
								(float)pixelValue,
								K4A_CALIBRATION_TYPE_DEPTH,
								K4A_CALIBRATION_TYPE_DEPTH,
								&ray, &valid))
							{
								printf("ERROR: calibration contained invalid transformation parameters. \n");
								goto Exit;
							}
							//��Ч�������ļ�
							VERTEX3D t;
							if (valid) {
								//fprintf(fp, "%d %d %d\n", -(int)ray.xyz.x, -(int)ray.xyz.y, (int)ray.xyz.z);
								t.x = -(int)ray.xyz.x; t.y = -(int)ray.xyz.y; t.z = (int)ray.xyz.z;
								g_vet.push_back(t);
								g_vet_color.push_back(v_rgb);
								i++;
							}
							//ͳ����Ƚ�С�ĵ�ĸ�����˵�������壩
							if (pixelValue < 2000) {
								count_depth_near++;
							}
						}
					}
				}

				/*FILE* fp_count = NULL;
				string outpath_count = "./count.txt";
				fp_count = fopen(outpath_count.c_str(), "a");
				fprintf(fp_count,"%d:%d\n",no_frame,count_depth_near);
				fclose(fp_count);*/

				FILE* fp = NULL;
				string outpath_txt = "PointCloudData\\" + filename + "\\";

				if (count_depth_near > 120000) {
					//������ͼ�ӽǵĲ�ɫͼ
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

					//���txt����
					if (0 != _access(outpath_txt.c_str(), 0)) {
						_mkdir(outpath_txt.c_str());
					}
					string outfile_txt = outpath_txt + to_string(no_frame) + ".txt";
					fp = fopen(outfile_txt.c_str(), "w");//����ĿĿ¼������ļ���
					//fprintf(fp, "%d\n", i); // ��������
					for (int j = 0; j < i; j++)
					{
						//fprintf(fp, "%d %d %d\n", g_vet[j].x, g_vet[j].y, g_vet[j].z);
						fprintf(fp, "%d %d %d %f %f %f\n", g_vet[j].x, g_vet[j].y, g_vet[j].z, ((float)g_vet_color[j].r / 255), ((float)g_vet_color[j].g / 255), ((float)g_vet_color[j].b / 255));
					}
					fclose(fp);
				}
				//��ʱ�ѿ����������ͼ���д�������opencv��֧����ʾ8λ�Ҷ�ͼ����Ҫ���ӻ��������һ��ת��
			}
			k4a_capture_release(capture);

			// Get a next capture
			switch (k4a_playback_get_next_capture(handle, &capture))
			{
			case K4A_WAIT_RESULT_SUCCEEDED:
				cur_frame++;

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