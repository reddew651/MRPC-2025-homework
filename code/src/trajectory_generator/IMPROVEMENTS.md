# 四旋翼飞行器规划与控制系统改进文档

## 概述

本项目对原有的四旋翼无人机规划与控制系统进行了以下改进：

1. **前端路径规划改进** - 新增 RRT* 采样式规划器
2. **后端轨迹优化改进** - 实现 Minimum Snap 轨迹优化
3. **控制器改进** - 集成姿态环和角速度环，完整的控制链

---

## 1. 前端路径规划改进

### 1.1 原有算法
- **A*** - 基础图搜索算法
- **JPS (Jump Point Search)** - 跳点搜索，优化的A*

### 1.2 新增算法: RRT* (Rapidly-exploring Random Tree Star)

**文件位置:**
- 头文件: `include/rrt_star_searcher.h`
- 源文件: `src/rrt_star_searcher.cpp`

**算法特点:**
- 概率完备性：随着采样点增多，一定能找到路径（如果存在）
- 渐进最优性：随着迭代次数增加，路径代价趋近最优
- 适合高维空间和复杂障碍物环境

**关键参数:**
```cpp
step_size_ = 0.5;           // 扩展步长
goal_threshold_ = 0.5;      // 到达目标阈值
rewire_radius_ = 1.5;       // 重连接半径
max_iterations_ = 5000;     // 最大迭代次数
goal_bias_ = 0.15;          // 目标偏置概率
```

**使用方法:**
```xml
<param name="planning/planner_type" value="2"/>  <!-- 0:A*, 1:JPS, 2:RRT* -->
```

---

## 2. 后端轨迹优化改进

### 2.1 原有算法: Polynomial QP
基于多项式的二次规划，满足边界条件和连续性约束。

### 2.2 新增算法: Minimum Snap 轨迹优化

**文件位置:**
- 头文件: `include/minimum_snap_trajectory.h`
- 源文件: `src/minimum_snap_trajectory.cpp`

**算法特点:**
- 最小化snap（加加速度的导数）的积分平方
- 生成更平滑的轨迹，减少执行器抖动
- 支持物理可行性约束（速度、加速度限制）
- 自动时间分配优化

**改进的时间分配算法:**
1. **梯形速度规划** - 考虑加速/减速段
2. **转角感知** - 根据路径转角调整速度
3. **迭代优化** - 自动调整时间使轨迹可行

**使用方法:**
```xml
<param name="planning/traj_opt_type" value="1"/>  <!-- 0:Poly QP, 1:Minimum Snap -->
```

---

## 3. 控制器改进

### 3.1 原有控制器: SO3Control
- 位置控制环（外环）
- 输出期望力和姿态四元数

### 3.2 改进控制器: ImprovedSO3Control

**文件位置:**
- 头文件: `include/so3_control/ImprovedSO3Control.h`
- 源文件: `src/ImprovedSO3Control.cpp`

**改进内容:**

#### 3.2.1 完整控制链
```
位置控制(外环) → 姿态控制(内环) → 角速度控制 → 电机混控
```

#### 3.2.2 姿态控制环（从simulator移植）
```cpp
// 姿态误差计算 (SO3上的误差)
Eigen::Vector3d eR = computeOrientationError(R, Rd);

// 力矩计算
torque_ = -kR_.asDiagonal() * eR 
          - kOm_.asDiagonal() * eOm 
          + omega_.cross(J_ * omega_);  // 陀螺力矩补偿
```

#### 3.2.3 积分项和抗饱和
```cpp
// 位置积分器
pos_integral_ += pos_error * dt;
saturate(pos_integral_, integral_limit_);

// 带积分的控制力
force_ = kp * pos_error + kv * vel_error + ki * pos_integral_ + ...
```

#### 3.2.4 Jerk前馈
```cpp
if (use_jerk_feedforward_) {
    force_ += mass_ * 0.02 * des_jerk;  // 改善动态响应
}
```

#### 3.2.5 电机混控输出
```cpp
// 从推力和力矩计算电机转速
w_sq(0) = thrust / (4*kf) - M2 / (2*d*kf) + M3 / (4*km);
w_sq(1) = thrust / (4*kf) + M2 / (2*d*kf) + M3 / (4*km);
w_sq(2) = thrust / (4*kf) + M1 / (2*d*kf) - M3 / (4*km);
w_sq(3) = thrust / (4*kf) - M1 / (2*d*kf) - M3 / (4*km);
```

---

## 4. 使用指南

### 4.1 编译
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 4.2 运行改进版本
```bash
# 默认配置 (JPS + Minimum Snap)
roslaunch trajectory_generator demo_improved.launch

# 使用RRT*规划器
roslaunch trajectory_generator demo_improved.launch planner:=2

# 使用A*规划器 + 原版优化器
roslaunch trajectory_generator demo_improved.launch planner:=0 optimizer:=0

# 调整飞行速度
roslaunch trajectory_generator demo_improved.launch max_vel:=5.0 max_acc:=4.0
```

### 4.3 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| planner | 1 | 规划器类型 (0:A*, 1:JPS, 2:RRT*) |
| optimizer | 1 | 优化器类型 (0:Poly QP, 1:Minimum Snap) |
| max_vel | 4.0 | 最大速度 (m/s) |
| max_acc | 3.0 | 最大加速度 (m/s²) |

---

## 5. 改进效果对比

### 5.1 规划性能
| 算法 | 计算时间 | 路径质量 | 适用场景 |
|------|----------|----------|----------|
| A* | 中等 | 一般 | 通用 |
| JPS | 快 | 一般 | 规则环境 |
| RRT* | 慢 | 优秀 | 复杂环境 |

### 5.2 轨迹质量
| 优化器 | 平滑度 | 执行器负载 | 跟踪精度 |
|--------|--------|------------|----------|
| Poly QP | 一般 | 中等 | 良好 |
| Minimum Snap | 优秀 | 低 | 优秀 |

### 5.3 控制性能
| 控制器 | 响应速度 | 稳定性 | 抗扰动 |
|--------|----------|--------|--------|
| 原版SO3 | 快 | 良好 | 一般 |
| 改进SO3 | 更快 | 优秀 | 优秀 |

---

## 6. 文件结构

```
trajectory_generator/
├── include/
│   ├── Astar_searcher.h           # 原有A*/JPS搜索器
│   ├── rrt_star_searcher.h        # 新增RRT*搜索器
│   ├── minimum_snap_trajectory.h  # 新增Minimum Snap优化器
│   └── trajectory_generator_waypoint.h
├── src/
│   ├── Astar_searcher.cpp
│   ├── rrt_star_searcher.cpp      # 新增
│   ├── minimum_snap_trajectory.cpp # 新增
│   ├── trajectory_generator_node.cpp      # 原版节点
│   └── trajectory_generator_improved.cpp  # 改进版节点
└── launch/
    ├── demo.launch                # 原版启动文件
    └── demo_improved.launch       # 改进版启动文件

so3_control/
├── include/so3_control/
│   ├── SO3Control.h               # 原有控制器
│   └── ImprovedSO3Control.h       # 改进控制器
└── src/
    ├── SO3Control.cpp
    ├── ImprovedSO3Control.cpp     # 新增
    └── so3_control_nodelet.cpp
```

---

## 7. 新颖性总结

1. **采样式规划与图搜索规划的对比研究** - 实现RRT*与A*/JPS的统一接口，便于性能对比
2. **Minimum Snap轨迹优化** - 生成更适合四旋翼执行的平滑轨迹
3. **完整控制链集成** - 将分散在simulator中的控制逻辑整合到控制器中
4. **自适应时间分配** - 根据路径特征和物理约束自动优化时间分配
5. **模块化设计** - 规划器和优化器可通过launch参数灵活切换

---

## 8. 潜在改进方向

1. 实现 Hybrid A* 用于考虑运动学约束的规划
2. 添加动态障碍物避障能力
3. 实现基于MPC的轨迹跟踪控制
4. 添加视觉/传感器融合的状态估计
