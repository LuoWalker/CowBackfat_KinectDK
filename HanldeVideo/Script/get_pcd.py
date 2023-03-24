import open3d as o3d
import numpy as np

def txt_pcd(name_txt, name_pcd):
	# 打开文件
	path = "./PointCloudData/" + name_txt
	source = o3d.geometry.PointCloud()
	m1 = np.loadtxt(path)[:, 0:3]
	source.points = o3d.utility.Vector3dVector(m1)

	# 可视化
	origin = source.get_center()  # 坐标系中心
	coordinate = o3d.geometry.TriangleMesh.create_coordinate_frame(
		size=0.5, origin=origin - 0.5)  # 坐标系
	o3d.visualization.draw_geometries([source, coordinate],
										width=700,
										height=700,
										)

	# 保存点云信息
	print("准备输出文件")
	o3d.io.write_point_cloud("./../../PCD/" + name_pcd, source)


# def main():
#    txt_pcd("1.txt", "test.pcd")


# if __name__ == "__main__":
#     main()