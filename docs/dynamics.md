# 动力学模块说明

## 软件边界

`drone_core` 是 ROS 无关的 C++17 动态库，只依赖 Eigen 和 C++ 标准库。其输入、状态和输出均为普通结构体或 Eigen 类型。`drone_dynamics` 是 ROS2 适配层，负责：

- 将 `drone_msgs/msg/MotorRPM` 转换为 rad/s；
- 从 ROS parameter 构造 `QuadrotorParameters`；
- 以固定步长调用 `QuadrotorModel::step()`；
- 发布 Odometry、IMU、实际 RPM、Path 和 TF；
- 提供命令超时保护和状态重置服务。
- 将配置的外力和外力矩作为显式扰动传入核心模型。

ROS2 适配层不包含推力、力矩、刚体运动或姿态积分公式。

## 坐标系

- 世界系：ENU，重力为 `[0, 0, -g]`；
- 机体系：FLU，总升力沿机体 `+z`；
- 四元数：将机体系向量旋转到世界系；
- 内部角速度单位：rad/s；
- ROS 电机命令单位：RPM。

X 型电机顺序：

| 编号 | 位置 | 旋转方向 | yaw 反扭矩符号 |
|---|---|---|---|
| 0 | 前左 `(+x,+y)` | CCW | 正 |
| 1 | 后左 `(-x,+y)` | CW | 负 |
| 2 | 后右 `(-x,-y)` | CCW | 正 |
| 3 | 前右 `(+x,-y)` | CW | 负 |

## 状态与方程

状态包括世界系位置和速度、机体到世界的姿态四元数、机体系角速度和四个实际电机角速度。

电机使用精确离散的一阶响应：

```text
omega_next = omega + (1 - exp(-dt/tau)) * (omega_cmd - omega)
```

单电机推力和反扭矩：

```text
F_i = k_F * omega_i^2
M_i = direction_i * k_M * omega_i^2
```

刚体运动：

```text
p_dot = v
v_dot = R(q) * [0, 0, T] / m + [0, 0, -g] - c_v * v / m
Omega_dot = I^-1 * (tau - c_Omega .* Omega - Omega x (I * Omega))
```

实现中还可叠加世界系外力 `F_disturbance` 和机体系外力矩 `tau_disturbance`。当前 `dynamics.yaml` 中二者均设为零，因此本阶段起飞悬停没有施加干扰。

平动和角速度使用半隐式 Euler。姿态使用机体系角速度形成增量四元数并右乘，之后每步归一化。

## 保护机制

- 命令和实际电机转速上下限；
- 非有限参数、状态、命令和时间步检测；
- 0.5 秒无新命令后自动将目标转速置零；
- 简化地面穿透约束；
- `/drone/reset` 恢复初始状态并清空路径；
- 历史路径点数上限。

## 默认悬停点

默认参数为 `m=1 kg`、`k_F=1.91e-6 N/(rad/s)^2`。理论单电机悬停速度为：

```text
omega_hover = sqrt(m*g/(4*k_F)) = 1133.15 rad/s
RPM_hover = 10820.8 RPM
```

节点启动时会根据实际加载参数重新计算并打印该数值。

## 已验证内容

纯核心单元测试覆盖：

1. 理论悬停推力和零合力矩；
2. 电机一阶响应；
3. X 型布局的 roll、pitch、yaw 力矩符号；
4. 电机转速限幅；
5. 长时间积分后的四元数归一化；
6. 地面约束；
7. 非法输入拒绝。

ROS2 运行检查已验证 launch、参数加载、Odometry、IMU、Path、TF、RPM 反馈、命令超时和 reset 服务。

## 当前限制

- 仅包含刚体、线性阻力和角阻尼，没有桨叶挥舞、地效或机架柔性；
- 地面为简化的非穿透约束，不是接触动力学；
- IMU 当前为无噪真值；
- 固定仿真步长由 wall timer 驱动，暂未实现暂停和仿真时钟；
- 当前已接入模型控制器；传感器噪声和扰动调度器尚未实现。

