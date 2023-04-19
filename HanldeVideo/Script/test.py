# %%
# 引入依赖
import open3d as o3d

# %%
# 可视化点云
path = "../PointCloudData/03219.mkv/144.txt"
source = o3d.io.read_point_cloud(path, format='xyzrgb')
o3d.visualization.draw_geometries([source], width=500, height=500)

# %%
# 保存点云信息
o3d.io.write_point_cloud("../../PCD/03219-1.pcd",
                         source,
                         write_ascii=True)

# %%
