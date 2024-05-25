import numpy as np
import open3d as o3d

import cv2


# 创建 Open3D 的 KinectRecorder 对象来读取陀螺仪数据
kinect_recorder = o3d.io.KinectRecorder("E:/luowenkuo/Video/1033/.mkv")

# 读取陀螺仪数据
gyro_data = kinect_recorder.get_gyroscope_data()

# 将陀螺仪数据积分得到角速度
# angular_velocity = integrate_gyroscope_data(gyro_data)


# 读取点云数据
point_cloud = o3d.io.read_point_cloud("your_point_cloud.ply")

# 读取陀螺仪数据
gyro_data = np.loadtxt("your_gyroscope_data.txt", delimiter=",")

# 陀螺仪数据的时间戳
gyro_timestamps = gyro_data[:, 0]  # 假设第一列是时间戳
gyro_values = gyro_data[:, 1:]  # 假设剩下的列是陀螺仪数据

# 点云数据的时间戳
point_cloud_timestamps = np.arange(len(point_cloud.points)) / 30.0  # 30帧每秒

# 对每一帧点云数据，找到最接近的陀螺仪数据的时间戳
gyro_values_for_point_cloud = []
for timestamp in point_cloud_timestamps:
    closest_index = np.argmin(np.abs(gyro_timestamps - timestamp))
    gyro_values_for_point_cloud.append(gyro_values[closest_index])

# 将陀螺仪数据与点云数据对应起来
# 现在，gyro_values_for_point_cloud 中的每个元素对应于点云数据的每一帧
