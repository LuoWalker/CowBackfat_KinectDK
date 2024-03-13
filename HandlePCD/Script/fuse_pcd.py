import open3d as o3d
import numpy as np
import os


def preprocess_point_cloud(pcd, voxel_size):
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(voxel_size, 30))

    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down, o3d.geometry.KDTreeSearchParamHybrid(voxel_size, 100)
    )
    return pcd, pcd_down, pcd_fpfh


def registration_feature(source, target, source_fpfh, target_fpfh, voxel_size):
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source,
        target,
        source_fpfh,
        target_fpfh,
        True,
        voxel_size * 1.5,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        4,
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                voxel_size * 3
            ),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.8),
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999),
    )

    return result


def registration_fast_global(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source,
        target,
        source_fpfh,
        target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold
        ),
    )
    return result


def registration_m_icp(source, target, result):
    # 参数设置
    voxel_size = 10  # 设置一个较小的体素大小作为最精细尺度
    current_transformation = result.transformation  # 初始化变换矩阵

    # 多尺度迭代
    for scale in range(3):
        # 下采样
        source_down = source.voxel_down_sample(voxel_size * (2**scale))
        target_down = target.voxel_down_sample(voxel_size * (2**scale))

        # ICP 配准
        result_icp = o3d.pipelines.registration.registration_icp(
            source_down,
            target_down,
            voxel_size * 1.5,  # 设置一个稍大的体素大小
            current_transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=200
            ),
        )

        # 更新变换矩阵
        current_transformation = result_icp.transformation
    return result_icp


def refine_registration(result_global, source, target, voxel_size):
    # result = o3d.pipelines.registration.registration_icp(
    #     source,
    #     target,
    #     distance_threshold,
    #     result_global.transformation,
    #     o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    # )
    result = o3d.pipelines.registration.registration_generalized_icp(
        source,
        target,
        voxel_size,
        result_global.transformation,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-06,
            relative_rmse=1e-06,
            max_iteration=5000,
        ),
    )  # 设置最大迭代次数

    return result


def draw_registration_result(source, target):
    cor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1000, origin=[0, 0, 0])

    vis1 = o3d.visualization.Visualizer()
    vis1.create_window(window_name="merge", width=960, height=540, left=0, top=100)
    vis1.add_geometry(source)
    vis1.add_geometry(cor)

    vis2 = o3d.visualization.Visualizer()
    vis2.create_window(window_name="gather", width=960, height=540, left=960, top=100)
    vis2.add_geometry(target)
    vis2.add_geometry(cor)

    while True:
        vis1.update_geometry(source)
        if not vis1.poll_events():
            break
        vis1.update_renderer()

        vis2.update_geometry(target)
        if not vis2.poll_events():
            break
        vis2.update_renderer()

    vis1.destroy_window()
    vis2.destroy_window()


# 读取多帧点云
pcd_path = "../../PCD/object/051311/"
pcd_files = os.listdir(pcd_path)[:11]
point_clouds = [o3d.io.read_point_cloud(pcd_path + pcd_file) for pcd_file in pcd_files]
print(pcd_files)

# 选择一个点云作为参考
voxel_size = 50
merged_point_cloud = o3d.geometry.PointCloud()
gathered_point_cloud = o3d.geometry.PointCloud()

target = point_clouds[0]
target.paint_uniform_color([0, 0, 1.0])
# draw_registration_result(point_clouds[0], point_clouds[9])
target, target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
merged_point_cloud += target
gathered_point_cloud += target

# 逐帧与参考点云进行配准
for index, source in enumerate(point_clouds[1:]):
    source, source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    # 全局
    result_global = registration_feature(
        source_down, target_down, source_fpfh, target_fpfh, voxel_size
    )
    # result_global = registration_fast_global(
    #     source_down, target_down, source_fpfh, target_fpfh, voxel_size
    # )
    print("-----%d-----" % (index))
    print("global:")
    print("fitness:%.3f,rmse:%.3f" % (result_global.fitness, result_global.inlier_rmse))
    print(result_global.transformation)
    source.transform(result_global.transformation)
    gathered_point_cloud += source

    # 局部
    result_local = refine_registration(result_global, source, target, voxel_size)
    # result_local = registration_m_icp(source, target, result_global)
    # result_local = registration_icp(source, target)
    print("local:")
    print("fitness:%.3f,rmse:%.3f" % (result_local.fitness, result_local.inlier_rmse))
    print(result_local.transformation)
    source.transform(result_local.transformation)
    merged_point_cloud += source


# # 将所有配准后的点云合并
# for pcd in point_clouds:
#     merged_point_cloud += pcd

# 比较融合前后的点云密度
print("Point Cloud Density Before Fusion:", len(target.points))
print("Point Cloud Density After Fusion:", len(merged_point_cloud.points))

# 可视化结果
draw_registration_result(merged_point_cloud, gathered_point_cloud)
