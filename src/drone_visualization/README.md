# drone_visualization

`drone_marker_node.py` 在 `base_link` 下以 Reliable + Transient Local QoS 发布机身、X 型机臂、四个旋翼和机头方向 Marker，并通过 `frame_locked` 跟随实时 TF；目标点在参考值变化时于 `map` 下发布。机体默认以 1 Hz 低频刷新，使 RViz 在 TF 建立较晚或重新启动后仍能恢复模型，同时避免旧版 20 Hz 刷新造成消息过滤器状态抖动。实际轨迹直接由 RViz2 的 Path display 显示 `/drone/path`。
