import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# 假设文件路径
file_path = "config/cube_region_model.bin"

# 读取二进制文件
with open(file_path, "rb") as f:
    # 解析 n_views (假设 size_t 为 8 字节)
    n_views = struct.unpack("<I", f.read(4))[0]
    print(f"Number of views: {n_views}")    # 2562

    views = []
    
    num_points = 200
    for _ in range(n_views):
        # 读取 data_points (假设 DataPoint 是 3 个 float)
        data_points = struct.unpack(f"{num_points * (152 // 4)}f", f.read(num_points * 152))
        data_points = np.array(data_points).reshape(-1, 152 // 4)

        # 读取 orientation (假设 orientation 是 3 个 float)
        orientation = struct.unpack("3f", f.read(3 * 4))
        orientation = np.array(orientation)  # 也转换为 NumPy 数组

        # 存储数据
        views.append({"data_points": data_points, "orientation": orientation})

data_points = views[0]["data_points"]
print(data_points.shape)  # (200, 3)

# 选择要可视化的 view
view_index = 0  # 选择第 0 个 view
selected_view = views[view_index]["data_points"]

# 可视化
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")

# 绘制散点图
ax.scatter(selected_view[:, 0], selected_view[:, 1], selected_view[:, 2], c="b", marker="o", alpha=0.6)

# 设置坐标轴标签
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title(f"3D Visualization of View {view_index}")

# 显示图像
plt.show()
