# drone_visualization

`drone_marker_node.py` 在 `base_link` 下以 Transient Local QoS 一次性发布机身、X 型机臂、四个旋翼和机头方向 Marker，并通过 `frame_locked` 跟随实时 TF；目标点在参考值变化时于 `map` 下发布。实际轨迹直接由 RViz2 的 Path display 显示 `/drone/path`。一次性发布避免静态模型反复进入 RViz TF 消息过滤器而产生闪烁。
