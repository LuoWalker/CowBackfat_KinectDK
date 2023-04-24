import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("../test.pcd")
coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100,
                                                         origin=[0, 0, 0])
o3d.visualization.draw_geometries([coor, pcd], width=500, height=500)
