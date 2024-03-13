import os
import concurrent.futures
import open3d as o3d
import numpy as np


def txt_pcd(file):
    filename = file.split("\\")[-1]
    # 保存点云信息
    source = o3d.geometry.PointCloud()
    xyz = np.loadtxt(file)[:, 0:3]
    # color = np.loadtxt(txt_path)[:, 3:6]
    source.points = o3d.utility.Vector3dVector(xyz)
    # source.colors = o3d.utility.Vector3dVector(color)

    record_name = file.split("\\")[-2].split(".")[0]

    pcd_path = "../../PCD/origin/" + record_name + "/"
    if not os.path.exists(pcd_path):
        os.makedirs(pcd_path)

    pcd_name = record_name + "-" + filename.replace(".txt", ".pcd")

    if o3d.io.write_point_cloud(pcd_path + pcd_name, source, write_ascii=True):
        print("保存成功" + pcd_name)
    else:
        print("保存失败" + pcd_name)


if __name__ == "__main__":
    # Replace 'your_folder' with the path to your folder containing point cloud files
    for folder_name in [
        "18315",
        "19137",
        "19161",
        "19161_2",
        "19181",
        "19235",
        "19257",
        "19259",
    ]:
        folder_path = "..\\PointCloudData\\" + folder_name

        # List all files in the folder
        files = [
            os.path.join(folder_path, f)
            for f in os.listdir(folder_path)
            if f.endswith(".txt")
        ]

        # Use ThreadPoolExecutor for concurrent processing
        with concurrent.futures.ThreadPoolExecutor() as executor:
            # Process each file concurrently
            executor.map(txt_pcd, files)
