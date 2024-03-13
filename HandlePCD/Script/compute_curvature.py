import open3d as o3d
import numpy as np
import os
from scipy.spatial import KDTree


def compute_curvature(pcd):
    # 计算点到点云的距离
    distances = np.asarray(pcd.compute_point_cloud_distance(pcd))

    # 根据距离计算曲率（这只是一个示例，实际情况可能需要自定义的曲率计算方法）
    curvature = 1.0 / (1 + distances)

    # 可视化
    # o3d.visualization.draw_geometries([pcd])

    return curvature


pcd_path = "../PCD/object/051311/"
pcd_files = os.listdir(pcd_path)[:11]
point_clouds = [o3d.io.read_point_cloud(pcd_path + pcd_file) for pcd_file in pcd_files]
print(pcd_files)

for pcd in point_clouds:
    # 估计法线
    pcd.estimate_normals()

    # 计算曲率
    curvature = compute_curvature(pcd)
    # curvature = np.linalg.norm(normals, axis=1)
    print(str(curvature))

    # 筛选出曲率较大的点
    threshold = 1.0
    high_curvature_points = pcd.select_by_index(np.where(curvature > threshold)[0])

    # 可视化
    o3d.visualization.draw_geometries([high_curvature_points])
