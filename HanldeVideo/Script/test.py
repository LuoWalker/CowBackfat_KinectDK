# %%
# 引入依赖
import open3d as o3d


# %%
# 自定义可视化控件以调整方向
def custom_draw_geometries(pcd):
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=700, height=700)
    vis.add_geometry(pcd)

    ctr = vis.get_view_control()
    ctr.set_front((0, 0, -1))

    vis.run()


# %%
# 可视化点云
path = "../PointCloudData/0321-1.ply"
pcd = o3d.io.read_point_cloud(path)
o3d.io.write_point_cloud("../../../PCD/" + "0321-1.pcd", pcd)
# o3d.visualization.draw_geometries([pcd], width=500, height=500)
custom_draw_geometries(pcd)

# %%
# 保存点云信息
o3d.io.write_point_cloud("../PointCloudData/background.ply", pcd)

# %%
# 为其他点云去除背景
target = o3d.io.read_point_cloud("../PointCloudData/background.ply")
source = o3d.io.read_point_cloud("../PointCloudData/0321-2.ply")
# o3d.visualization.draw_geometries([source, target], width=700, height=700)
# o3d.visualization.draw_geometries([target], width=700, height=700)
o3d.visualization.draw_geometries([source], width=700, height=700)

# %%
def get_mesh(_relative_path):
    mesh = o3d.io.read_triangle_mesh(_relative_path)
    mesh.compute_vertex_normals()
    return mesh

print("->Ball pivoting...")
_relative_path = "../PointCloudData/0321-2.ply"    # 设置相对路径
N = 2000                        # 将点划分为N个体素
pcd = get_mesh(_relative_path).sample_points_poisson_disk(N)
o3d.visualization.draw_geometries([pcd])

radii = [0.005, 0.01, 0.02, 0.04]
rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
o3d.visualization.draw_geometries([pcd, rec_mesh])
# %%
