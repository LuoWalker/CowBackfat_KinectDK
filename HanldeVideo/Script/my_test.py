# %%
# 导入依赖
import open3d as o3d
import numpy as np

# %%
# 可视化点云
pcd = o3d.io.read_point_cloud("../../PCD/origin/0310/1058/1058-11.pcd")
coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1000,
                                                         origin=[0, 0, 0])
o3d.visualization.draw_geometries([coor, pcd], width=700, height=700)

# %%
# 倒置点云
# pcd = o3d.io.read_point_cloud("../../PCD/newBackground.pcd")
# for point in pcd.points:
#     point[2] = -point[2]
# o3d.io.write_point_cloud("../../PCD/newBackground.pcd", pcd, write_ascii=True)

# %%
