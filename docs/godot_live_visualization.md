# Godot Live Visualization

这套实时可视化链路使用本机 UDP 将 MATLAB/Simulink 仿真状态发送到 Godot，再由 Godot 进行 3D 渲染。

## 当前实现

- MATLAB 端通过 `udpport` 以 JSON 数据包实时发送轨道与姿态状态
- Godot 端监听 `127.0.0.1:4242`，实时更新地球与卫星模型
- Godot 场景为程序化生成，不依赖额外模型资源

## 数据包格式

MATLAB 当前发送的字段：

- `sim_time_s`
- `position_i_m`
- `velocity_i_mps`
- `q_bi`
- `omega_b_radps`

其中 `q_bi` 默认按 `[w; x; y; z]` 解释。

## 启动方式

1. 在 Godot 中打开 `Visualization/project.godot`
2. 运行主场景 `res://scenes/main.tscn`
3. 在 MATLAB 中运行：`run_godot_stream_demo`

## 接入真实仿真

如果你已经有 Simulink 或 MATLAB 主循环，只需要在每个仿真步调用：

```matlab
stream = godot_stream_open('127.0.0.1', 4242);
godot_stream_send_state(stream, t, X_i, V_i, q_bi, omega_b);
```

如果你手里已经是 `StatesBus` 风格结构体，也可以直接发：

```matlab
godot_stream_send_states_bus(stream, t, states);
```

建议把这一步放在：

- MATLAB 数值积分循环末尾
- Simulink 的 MATLAB Function / System object 外层记录逻辑
- 仿真输出后处理回调中

## 后续建议

- 把太阳方向、地球自转、轨迹尾迹也一起发送到 Godot
- 将姿态参考、控制力矩、传感器测量做成额外图层
- 如需更高吞吐量，可把 JSON 改为二进制 UDP 包
