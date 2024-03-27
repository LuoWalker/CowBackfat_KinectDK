from collections import Counter

import open3d as o3d
import numpy as np
import os

dir = "../../PCD/origin/0310/"
pcd_paths = os.listdir(dir)
for pcd_path in pcd_paths:
    if pcd_path.startswith("0513") or pcd_path.startswith("0321"):
        continue
    out_path = "../../PCD/object/0310/" + pcd_path + "/"
    if not os.path.exists(out_path):
        os.mkdir(out_path)
    else:
        print(out_path + "已存在，跳过")
        continue
    pcd_files = os.listdir(dir + pcd_path + "/")
    # point_clouds = [o3d.io.read_point_cloud(pcd_path + pcd_file) for pcd_file in pcd_files]
    for pcd_file in pcd_files:
        point_cloud = o3d.io.read_point_cloud(dir + pcd_path + "/" + pcd_file)
        print(point_cloud)
        point_cloud = point_cloud.voxel_down_sample(voxel_size=5)
        print(point_cloud)

        # 密度聚类
        eps = 40  # 同一聚类中最大点间距
        min_points = 100  # 有效聚类的最小点数
        with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Warning
        ) as cm:
            labels = np.array(
                point_cloud.cluster_dbscan(eps, min_points, print_progress=False)
            )
        max_label = labels.max()
        print(
            f"point cloud has {max_label + 1} clusters"
        )  # label = -1 为噪声，因此总聚类个数为 max_label + 1

        # 使用 Counter 统计每个类别的点数
        label_counts = Counter(labels)

        # 找到具有最大点数的聚类
        label_counts = Counter(labels)

        most_common_label = label_counts.most_common(1)[0][0]

        # 提取最大点数的聚类的点云
        max_cluster_indices = np.where(labels == most_common_label)[0]
        max_cluster_points = point_cloud.select_by_index(max_cluster_indices)

        # 将最大点数的聚类保存为新的点云
        # max_cluster_points.paint_uniform_color([1, 0, 0])
        # o3d.visualization.draw_geometries([max_cluster_points])
        out_path = "../../PCD/object/0310/" + pcd_path + "/"
        if not os.path.exists(out_path):
            os.mkdir(out_path)
        o3d.io.write_point_cloud(
            out_path + pcd_file, max_cluster_points, write_ascii=True
        )
        print(out_path + pcd_file)
