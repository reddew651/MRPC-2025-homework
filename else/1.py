import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.spatial.transform import Rotation as R

# 设置绘图风格 (需要安装 seaborn: pip install seaborn)
sns.set_theme(style="whitegrid")

# ===========================
# 1. 参数设置 与 2. 数据读取（使用真实 tracking.csv）
# ===========================
omega = 0.5
alpha = np.pi / 12

# 读取与本脚本同目录下的 tracking.csv
base_dir = os.path.dirname(__file__)
tracking_path = os.path.join(base_dir, "tracking.csv")
df = pd.read_csv(tracking_path)

# CSV 列顺序: t, qx, qy, qz, qw
timestamps = df["t"].values
# SciPy 四元数顺序为 [x, y, z, w]
q_body_vals = df[["qx", "qy", "qz", "qw"]].values

# ===========================
# 3. 核心计算循环 (算法优化：连续性处理)
# ===========================
q_world_list = []
q_prev = None # 用于存储前一个时刻的四元数

for i, t in enumerate(timestamps):
    # --- 计算部分保持不变 ---
    r_body = R.from_quat(q_body_vals[i])
    
    wt = omega * t
    ca, sa = np.cos(alpha), np.sin(alpha)
    cwt, swt = np.cos(wt), np.sin(wt)
    
    matrix_rel = np.array([
        [cwt, -swt * ca,  swt * sa],
        [swt,  cwt * ca, -cwt * sa],
        [0,    sa,        ca      ]
    ])
    r_rel = R.from_matrix(matrix_rel)
    r_world = r_body * r_rel
    q_curr = r_world.as_quat() # Scipy 返回 [x, y, z, w]
    
    # --- 符号选择：兼顾连续性与题目要求 qw >= 0 ---
    # 先用点积选择与上一时刻“更连续”的符号
    if q_prev is not None and np.dot(q_curr, q_prev) < 0:
        q_curr = -q_curr

    # 再强制满足 qw >= 0（题目硬性要求）
    if q_curr[3] < 0:
        q_curr = -q_curr
            
    q_world_list.append(q_curr)
    q_prev = q_curr # 更新前一个四元数

q_world_arr = np.array(q_world_list)

# ===========================
# 4. 绘图优化
# ===========================
plt.figure(figsize=(12, 7)) # 稍微加大画布

# 使用更深、更专业的颜色
colors = ['#d62728', '#2ca02c', '#1f77b4', '#000000'] # 深红, 深绿, 深蓝, 纯黑
labels = ['$q_x$', '$q_y$', '$q_z$', '$q_w$']
linewidth = 2.5 # 加粗线条

# Scipy quats are [x, y, z, w]
for k in range(4):
    plt.plot(timestamps, q_world_arr[:, k], label=labels[k], color=colors[k], linewidth=linewidth)

# 标题去掉了 (qw >= 0) 的约束说明，因为我们现在是保证连续性
plt.title('End-Effector Attitude Quaternion in World Frame (Continuous)', fontsize=16, fontweight='bold')
plt.xlabel('Time (s)', fontsize=14)
plt.ylabel('Quaternion Value', fontsize=14)

# 优化图例：增加背景框和阴影，放置在合适位置
plt.legend(loc='best', fontsize=12, frameon=True, shadow=True, fancybox=True)

# 优化刻度标签大小
plt.xticks(fontsize=12)
plt.yticks(fontsize=12)

# 设置Y轴范围，稍微留点边距
plt.ylim(-1.1, 1.1)

# 网格线已经由 seaborn 设置好了，无需手动设置 alpha
plt.tight_layout()

plt.savefig('quaternion_plot_optimized.png', dpi=300)
plt.show()