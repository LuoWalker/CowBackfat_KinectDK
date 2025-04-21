import os
import open3d as o3d
import numpy as np


def txt_pcd(txt_dir, start_frame):
    # 遍历所有txt点云，并批量转为PCD
    # if (len(txt_dir) == 1):
    dir_name = txt_dir
    path = "./PointCloudData/" + dir_name
    filenames = os.listdir(path)
    for filename in filenames:
        # if (int(filename) >= start_frame
        #        and int(filename) <= start_frame + 30):
        txt_path = path + "/" + filename
        # 保存点云信息
        source = o3d.geometry.PointCloud()
        xyz = np.loadtxt(txt_path)[:, 0:3]
        rgb = np.loadtxt(txt_path)[:, 3:6]
        source.points = o3d.utility.Vector3dVector(xyz)
        source.colors = o3d.utility.Vector3dVector(rgb)

        record_name = dir_name.split(".")[0]

        pcd_path = "./../PCD/origin/0310/" + record_name + "/"
        if not os.path.exists(pcd_path):
            os.makedirs(pcd_path)

        pcd_name = record_name + "-" + filename.replace(".txt", ".pcd")

        if o3d.io.write_point_cloud(pcd_path + pcd_name, source, write_ascii=True):
            print("保存成功" + pcd_name)
        else:
            print("保存失败" + pcd_name)


# if (len(txt_dir) == 0):
#     path = "./PointCloudData"
#     for dirpath, dirnames, filenames in os.walk(path):
#         for dirname in dirnames:
#             filenames = os.listdir(dirpath + '/' + dirname)
#             for filename in filenames:
#                 txt_path = dirpath + '/' + dirname + '/' + filename
#                 # 保存点云信息
#                 source = o3d.geometry.PointCloud()
#                 xyz = np.loadtxt(txt_path)[:, 0:3]
#                 color = np.loadtxt(txt_path)[:, 3:6]
#                 source.points = o3d.utility.Vector3dVector(xyz)
#                 source.colors = o3d.utility.Vector3dVector(color)

#                 pcd_path = dirname[:4] + '-' + filename.replace(
#                     'txt', 'pcd')
#                 if (o3d.io.write_point_cloud("./../PCD/origin/" + pcd_path,
#                                              source,
#                                              write_ascii=True)):
#                     print("保存成功" + pcd_path)
#                 else:
#                     print("保存失败" + pcd_path)

if __name__ == "__main__":
    txt_pcd("1123.mkv", 6)
