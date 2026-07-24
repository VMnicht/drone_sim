# 与 pengyu_sim / MARSIM 的设计对比

本文只做架构与实验方法对比，不复制两个参考项目的动力学、控制或点云渲染代码。对比时间为 2026-07-15；`pengyu_sim` 检查到提交 `a15b0992`（2026-05-14），MARSIM 以公开仓库和论文为准。

## 1. 来源

- [pengyu_sim（Gitee）](https://gitee.com/potato77/pengyu_sim)
- [MARSIM（GitHub）](https://github.com/hku-mars/MARSIM)
- [MARSIM 论文（arXiv:2211.10716）](https://arxiv.org/abs/2211.10716)

## 2. 总体架构

| 维度 | 本工程 | pengyu_sim | MARSIM |
|---|---|---|---|
| 中间件 | ROS2 Humble、colcon、Python launch | ROS1、catkin | 主分支 ROS1/catkin，仓库另有 ROS2 分支 |
| 主要目标 | 可解释的小型四旋翼全链路、调参与自动验收 | UAV/UGV/空地平台动力学，并模拟 MAVROS/PX4 接口以联调 Sunray | 面向 LiDAR UAV 的真实点云地图与高保真扫描渲染 |
| 动力学/控制边界 | `drone_core` 只依赖 Eigen/STL，ROS 节点为适配层 | UAV 动力学类与 ROS 节点分文件，但工作区整体以 ROS1 接口组织 | 论文框架包含内置控制、动力学/运动学与 LiDAR 三个模块，均通过 ROS 接外部 SLAM/规划 |
| 地图/感知 | YAML 几何体→表面采样→视场/遮挡/噪声点云→体素 | `map_generator` 和 `local_sensing`；仓库说明其主要源于 MARSIM | 直接导入真实 PCD，体素筛选、深度图插值、遮挡剔除、平面修正，可选 OpenGL GPU |
| 规划 | 自研有限边界 3D voxel A*、膨胀、直线简化和跟踪 | 主要作为 Sunray 规划联调环境 | 重点提供仿真环境，可外接 FUEL 等规划器 |
| 多机 | 3 个独立 ROS2 namespace/TF/传感器种子，统一安全间距监测 | `agent_name`、`agent_id`、`swarm_num` 参数化多个平台 | 每架 UAV 可作为独立 ROS 节点/线程甚至分布到多台计算机，并互相进入 LiDAR 扫描 |
| 自动验收 | 11 场景 JSON/CSV/图、阈值检查、回放、3×3 参数扫描 | 仓库含若干离线评估程序，README 仍把闭环响应验证列为 TODO | 论文主要评测 LiDAR 渲染耗时、内存与真实飞行轨迹一致性 |

## 3. 动力学与控制差异

三者均使用四旋翼刚体思路，但工程重点不同。本工程把电机一阶响应、X 型 mixer、六自由度刚体、几何姿态控制、解析轨迹、扰动和传感器噪声全部放入 ROS 无关的 C++ 库，并允许不安装 ROS 直接用 CMake/CTest 验证。`pengyu_sim` 更强调对 MAVROS/PX4 语义的兼容：`fake_mavros_bridge_node` 接模式和解锁，`px4_control_sim_node` 把 MAVROS setpoint 转为 RPM，再交给 `quadrotor_dynamics_node`。MARSIM 的内置飞控与动力学服务于 LiDAR/规划实验，论文采用标准刚体模型和几何控制，但其核心创新是点云渲染，而不是控制软件解耦。

本工程的优点是边界清晰、参数可追溯和单元测试容易；代价是没有 PX4/MAVROS 的完整状态机，也没有 MARSIM 的真实飞控/实机同场对照。

## 4. 地图、点云与碰撞差异

MARSIM 的输入是数百万点级真实环境 PCD。论文描述的在线流程先按空间体素筛选视场内点，再投影到深度图，利用最小深度做遮挡处理，并对平面点做射线—平面修正；GPU 版本通过 OpenGL 并行渲染。碰撞检测用静态/动态点云 KD-Tree 做半径近邻查询。它支持多个真实 LiDAR 扫描模式、动态球形障碍物和其他 UAV 的互相观测。

本工程刻意选择更小的验收规模：障碍物由 YAML 的 box/cylinder 构成，在启动时采样表面点；局部感知按量程、水平/垂直视场、角度 bin 遮挡、噪声和丢点生成 `PointCloud2`，再量化为体素。碰撞安全由障碍物几何距离、无人机半径和 0.30 m 规划裕度共同保证。这样不需要 PCL/OpenGL/GPU，适合 WSL、CI 和控制算法调试，但不应称为点真实 LiDAR，也不能用于传感器扫描模式或真实地图细节研究。

`pengyu_sim` 当前同时包含几何动力学与从 MARSIM 衍生的 `map_generator/local_sensing` 路线，适合与现有 ROS1/Sunray 工程联调；本工程没有沿用其源码，而是在 ROS2 消息、QoS 和 YAML 场景体系内重新实现简化版本。

## 5. 实验结果对比方式

不同项目没有完全相同的地图、机体、控制器与硬件，因此不能把数值横向排名。MARSIM 论文报告的是点云渲染吞吐/内存及仿真—实飞轨迹相似性；本工程报告的是闭环控制与任务安全指标。当前可复现实测结果如下：

| 场景 | 本工程结果 | 说明 |
|---|---:|---|
| hover | 稳态误差 0.0189 m | 12 s、带默认传感器噪声 |
| circle / figure-eight | RMS 0.3778 / 0.2812 m | 包括起飞过渡段 |
| wind_gust | 3.041 N，恢复 2.077 s | 2 s 阵风，最终误差 0.0254 m |
| sensor_noise | 位置噪声标准差约 `[0.0304,0.0298,0.0501]` m | 与 YAML 的 `[0.03,0.03,0.05]` 一致 |
| motor fault | 100 个命令被修改，最终误差 0.0155 m | 0 号电机效率短时降至 82% |
| 5 obstacles / narrow / replan | 最小净间隙 0.440 / 0.399 / 0.481 m | 三个任务均完成，规划终态 `goal_reached`，且统一满足 0.30 m 净距合同 |
| 3 drones | 最小观测间距 0.786 m | YAML 要求不低于 0.75 m，0 次违规 |

所有数据来自 `artifacts/experiments/*/summary.json` 和多机验收脚本。MARSIM 论文中的“GPU 比 Gazebo 更快”等结论只适用于其指定的 PCD 分辨率、LiDAR 参数和 NUC/GPU 平台，本文不把它转写成本工程性能结论。

## 6. 取舍结论

- 如果目标是高保真真实点云 LiDAR、扫描模式复现与大场景感知研究，MARSIM 明显更合适。
- 如果目标是接入 ROS1 Sunray、模拟 MAVROS/PX4 语义并同时覆盖 UAV/UGV，pengyu_sim 更直接。
- 如果目标是 ROS2 Humble 下验证自研动力学/控制/规划、全部 YAML 调参、无界面批量验收和故障回放，本工程的依赖更轻、算法边界和证据链更完整。
- 下一阶段若追求感知真实性，应优先加入 PCD 导入、KD-Tree/体素索引和扫描模式，而不是继续增加几何障碍物数量；若追求飞控兼容，则应增加 MAVROS/PX4 bridge，而不应污染 `drone_core`。
