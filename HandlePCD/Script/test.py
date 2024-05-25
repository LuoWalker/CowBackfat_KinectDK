import open3d as o3d
import numpy as np

# 读取点云数据
cloud = o3d.io.read_point_cloud("../../PCD/")

# 转换为 NumPy 数组
cloud_array = np.asarray(cloud)

# 指定方向，假设选择了 x 方向
direction = "x"

# 根据选择的方向对点云数据进行排序
sorted_indices = np.argsort(cloud_array[:, "xyz".index(direction)])

# 初始化空列表，用于存储每组中的最高点
highest_points = []

# 初始化变量，用于跟踪当前组的起始索引
start_index = 0

# 迭代排序后的索引数组，对每个分组找到最高点
for i in range(1, len(sorted_indices)):
    if (
        cloud_array[sorted_indices[i]][direction]
        != cloud_array[sorted_indices[i - 1]][direction]
    ):
        # 当前组结束，找到当前组中的最高点
        end_index = i
        highest_point_index = np.argmax(
            cloud_array[sorted_indices[start_index:end_index]][:, "z"]
        )
        highest_point = cloud_array[sorted_indices[start_index:end_index]][
            highest_point_index
        ]
        highest_points.append(highest_point)
        # 更新起始索引
        start_index = i

# 最后一组
highest_point_index = np.argmax(cloud_array[sorted_indices[start_index:]][direction])
highest_point = cloud_array[sorted_indices[start_index:]][highest_point_index]
highest_points.append(highest_point)

# 打印结果
for point in highest_points:
    print(point)

# 使用最小二乘法拟合直线
# 构建增广矩阵
X = np.column_stack((highest_points[:, 0], np.ones(len(highest_points))))

# 计算最小二乘解
coefficients = np.linalg.lstsq(X, highest_points[:, 1:], rcond=None)[0]

# coefficients[0] 是斜率，coefficients[1] 是截距
slope, intercept = coefficients[0], coefficients[1]

line_points = np.array(
    [
        [min(cloud[:, 0]), slope * min(cloud[:, 0]) + intercept, 0],
        [max(cloud[:, 0]), slope * max(cloud[:, 0]) + intercept, 0],
    ]
)

# 打印结果
print("斜率:", slope)
print("截距:", intercept)

# 创建可视化窗口并添加点云和直线
visualizer = o3d.visualization.Visualizer()
visualizer.create_window()
visualizer.add_geometry(cloud)
visualizer.add_geometry(line_points)

# 设置渲染参数并显示窗口
render_options = visualizer.get_render_option()
render_options.point_size = 5
render_options.line_width = 2
visualizer.run()
visualizer.destroy_window()
