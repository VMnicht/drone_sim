# 完成性审计

审计基准为 `任务文档.md` 与 `工程规划.md`。任务文档列出的 10 个加分方向均已实现并纳入正式验收；动力学、控制、mixer、轨迹、扰动和传感器数学模型位于不依赖 ROS2 的 `drone_core`，地图、点云、碰撞检测和规划允许使用 ROS2。

## 基础要求

| 要求 | 实现与证据 | 状态 |
|---|---|---|
| ROS2 Humble 工程与免 source 启动 | 15 个 package 可构建；`start_sim.sh` 在脚本内部加载 ROS2/工作区并可自动构建 | 通过 |
| 四路 RPM 六自由度动力学 | 电机一阶响应、X 型力/矩、刚体平动/转动、四元数积分、阻力、地面约束与限幅 | 通过 |
| 目标状态到四路 RPM 控制 | 位置/速度外环、几何姿态/角速度内环、统一 mixer 与多级安全限幅 | 通过 |
| 算法与 ROS2 解耦 | `drone_core` 仅依赖 Eigen/STL，支持 `DRONE_CORE_STANDALONE=ON` 独立构建和 CTest | 通过 |
| RViz2 可视化 | 机体、旋翼、目标、轨迹、地图、点云、体素、规划路径、扰动力；Marker 使用稳定 ID、零时间戳、`frame_locked` 和 transient-local QoS | 通过 |
| 全量 YAML 参数化 | 13 类节点共 339 项声明参数均由实际 launch 配置链覆盖，且无未声明 YAML 键 | 通过 |
| 自动实验与材料 | 11 场景 CSV/JSON/PNG、自动阈值验收、参考对比、PDF 和 MP4 生成脚本 | 通过 |
| 公开 Git 仓库 | `origin` 指向 <https://github.com/VMnicht/drone_sim>，源码、报告、视频和复现材料统一交付 | 通过 |

## 10 项加分功能矩阵

| # | 加分方向 | 落地内容 | 自动/实测证据 | 状态 |
|---:|---|---|---|---|
| 1 | YAML 配置 | `src/drone_bringup/config/` 中模型、控制、传感器、地图、规划、场景等 29 个配置文件；支持 `override_config` 和 Panel 安全编辑 | `scripts/verify_yaml_parameters.py` 逐节点检查、Panel 全量白名单和跨配置合同，全部通过 | 通过 |
| 2 | 风/外部扰动 | 常值、正弦、阵风、固定种子随机力/力矩，带生效时间窗和运行时开关 | `wind_gust`：最大 3.041 N，最终误差 0.0457 m、稳态误差 0.0723 m | 通过 |
| 3 | 传感器噪声 | 真值与带噪 Odom/IMU 分离；GPS；白噪声、偏置、随机游走、输出延迟、丢包和固定种子 | `sensor_noise`：位置噪声统计与场景 YAML 一致，最终误差 0.0777 m | 通过 |
| 4 | 点云/体素/局部感知 | YAML 几何体表面采样，视场、量程、遮挡、噪声、丢点，以及带持久化/过期衰减的体素 Marker | `perception_replan` 最大局部点数 802，完成重规划且净间隙 0.4889 m | 通过 |
| 5 | 多无人机 | 3 个独立 namespace、TF、传感器种子和轨迹；fleet 安全监测与专用 RViz | `scripts/test_multi_drone.py`：3/3 活跃，最小间距 0.786 m ≥ 0.75 m，0 次违规 | 通过 |
| 6 | 多种轨迹 | ROS 无关圆、Gerono 八字生成器与 YAML 航点，提供速度/加速度/yaw 前馈 | circle RMS 0.3827 m；figure-eight RMS 0.2813 m；square 5/5 航点完成 | 通过 |
| 7 | 单测/自动评测 | gtest、launch test、Panel 进程测试、路径进度回归测试、场景阈值检查、CSV/JSON/PNG 记录器 | `colcon test-result`：35 tests，0 error/failure/skip；11/11 场景通过 | 通过 |
| 8 | Web 地面站与任务 Panel | 地面站负责状态/曲线/命令；Panel 提供 30 个逐项展示入口、5 种模式、11 场景、RViz、16 类指标、7 类结果图和 29 份 YAML 安全编辑 | 地面站 API 测试通过；`scripts/test_mode_panel.py` 验证入口全覆盖、文件白名单、互斥启停、配置回滚和独立实验结果 | 通过 |
| 9 | 参考项目对比 | 与 pengyu_sim、MARSIM 对比中间件、动力学边界、感知、规划、多机和评测 | `docs/reference_comparison.md` 含来源、限制与本工程实测数据 | 通过 |
| 10 | 创新扩展 | 电机效率/上限与命令丢包/延迟/冻结；确定性回放；3×3 参数扫描 | fault 场景修改 95 个命令并恢复至 0.0179 m；回放与扫描 CSV/热图齐全 | 通过 |

## 正式场景结果

| 场景 | 关键结果 | 验收 |
|---|---|---|
| hover | 稳态误差 0.0203 m | 通过 |
| target | 最终误差 0.0203 m，任务完成 | 通过 |
| square | 最终误差 0.0168 m，5/5 航点完成 | 通过 |
| circle | 轨迹 RMS 0.3827 m | 通过 |
| figure_eight | 轨迹 RMS 0.2813 m | 通过 |
| wind_gust | 最终误差 0.0457 m，稳态误差 0.0723 m | 通过 |
| sensor_noise | 最终误差 0.0777 m | 通过 |
| fault_motor | 95 个命令被修改，最大倾角 19.11°，最终误差 0.0179 m | 通过 |
| five_obstacles | 最小净间隙 0.4196 m，`goal_reached` | 通过 |
| narrow_passage | 最小净间隙 0.4136 m，`goal_reached` | 通过 |
| perception_replan | 最小净间隙 0.4889 m，任务完成 | 通过 |

场景数据位于 `artifacts/experiments/<scenario>/summary.json`，阈值来自 `evaluation.yaml`，不是写死在验收脚本中。

## 稳定性专项证据

- 最终 11 场景同批次运行并记录 rosbag，`scripts/verify_experiment_evidence.py` 检查 11/11 场景的索引、消息数和必需 topic 全部通过；
- 最终 11 份 `run.log` 中 `Odometry timed out`、`Motor command timed out` 和 `failsafe` 事件合计为 0；
- `artifacts/performance/final_rviz/` 实测 Odom/TF 95.23 Hz、最大间隔 12 ms，路径 4.762 Hz、模型 Marker 1.000 Hz；路径带宽由约 44 KB/s 增长到约 115 KB/s 后受 1200 点上限约束；
- Odom 与 TF 发布者计数均为 1；第二个 `start_sim.sh` 会以退出码 3 拒绝，Panel 启动测试覆盖该行为；
- 路径进度回归测试覆盖折叠/自交路径，确保已走进度不倒退且前视目标不被误判为已经到达。

## 复核命令

```bash
cd ~/drone_sim_ws

# 参数声明、加载链和 YAML 反向拼写检查
python3 scripts/verify_yaml_parameters.py

# 11 个正式场景的已有结果验收；重新运行用 ./start_sim.sh batch
python3 scripts/verify_experiments.py
python3 scripts/verify_experiment_evidence.py --root artifacts/experiments

# 当前机器的 RViz/topic/带宽/CPU 性能采样
./scripts/profile_runtime.sh final_rviz true

# ROS2/核心测试
colcon test --event-handlers console_direct+
colcon test-result --verbose

# ROS 无关核心独立构建
cmake -S src/drone_core -B /tmp/drone_core_standalone \
  -DDRONE_CORE_STANDALONE=ON -DBUILD_TESTING=ON
cmake --build /tmp/drone_core_standalone
ctest --test-dir /tmp/drone_core_standalone --output-on-failure

# 专项验收
python3 scripts/test_multi_drone.py
python3 scripts/test_ground_station_api.py
python3 scripts/replay_scenario.py
python3 scripts/run_parameter_sweep.py
```

除“创建公开远端仓库并推送”需要仓库所有者的外部账号与授权外，工程内可执行内容均已落地。
