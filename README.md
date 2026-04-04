# 卫星仿真项目

本仓库是一个按 MATLAB / Simulink 工程方式组织的卫星仿真原型项目，当前重点已经从纯目录骨架扩展到“环境参考模型 + 传感器测量链 + 姿态确定 / 姿态估计”这一条可运行的主线。

## 当前工程现状

当前仓库已经具备以下基础能力：

- 卫星六自由度仿真基础
  - 已具备卫星平动 + 转动的六自由度仿真基础结构
  - 可作为环境、传感器、姿态确定与控制算法的被控对象基础
- 环境参考模型
  - 太阳参考方向 `sun_i`
  - 地磁参考方向 `mag_i`
  - 空气阻力、太阳辐射压、重力梯度、磁扰动力矩等基础环境/扰动模型
- 姿态传感器链
  - 太阳敏感器理想测量模式
  - 粗太阳敏感器阵列原始值模拟与太阳方向重建
  - 磁力计理想测量模式
  - 三轴磁力计原始值模拟与标定恢复
- 姿态确定与估计
  - 基于太阳方向和地磁方向的 TRIAD 直接定姿
  - 基于四元数的姿态 EKF
  - 传感器测量打包与姿态估计封装接口
- 工程化支撑
  - 路径初始化脚本
  - baseline 配置脚本
  - smoke tests
  - Godot 可视化演示入口

## 当前重点模块

目前最完整、最适合继续往下扩展的是姿态估计相关链路：

- 环境参考模型
  - [calc_sun_vector_i.m](src/environment/calc_sun_vector_i.m)
  - [calc_magnetic_field_i.m](src/environment/calc_magnetic_field_i.m)
  - [environment_config.m](src/environment/environment_config.m)
- 传感器模型
  - [measure_sun_sensor.m](src/sensors/measure_sun_sensor.m)
  - [reconstruct_sun_vector_from_css.m](src/sensors/reconstruct_sun_vector_from_css.m)
  - [measure_magnetometer.m](src/sensors/measure_magnetometer.m)
  - [calibrate_magnetometer_raw.m](src/sensors/calibrate_magnetometer_raw.m)
  - [measure_attitude_sensors.m](src/sensors/measure_attitude_sensors.m)
- 观测器与姿态确定
  - [solve_attitude_from_sun_mag.m](src/observers/solve_attitude_from_sun_mag.m)
  - [triad_attitude_init.m](src/observers/triad_attitude_init.m)
  - [attitude_ekf_step.m](src/observers/attitude_ekf_step.m)
  - [estimate_attitude_from_sensors.m](src/observers/estimate_attitude_from_sensors.m)

## 环境模型模式

当前太阳参考方向和地磁参考方向支持多种模式切换，统一由 [environment_config.m](src/environment/environment_config.m) 配置：

- `simple`
  - 用于快速原型验证
- `engineering`
  - 用于更正式的工程近似建模
- `high_fidelity`
  - 预留给更高保真模型
  - 优先调用 MATLAB/Aerospace Toolbox 或用户自定义函数句柄
  - 若缺少依赖会显式报错，不会静默退化

## 目录结构

- `models/plant/`：卫星本体动力学与被控对象模型
- `models/mission/`：任务级场景模型、顶层仿真装配与工况变体
- `models/libraries/`：可复用的 Simulink 模块库和公共子系统
- `src/parameters/`：卫星、轨道、仿真参数与总线定义
- `src/environment/`：太阳、磁场、重力、气动等环境参考与扰动模型
- `src/gnc/`：姿态表示、GNC 配置与相关基础算法
- `src/sensors/`：太阳敏感器、磁力计及原始值重建/标定逻辑
- `src/observers/`：TRIAD、EKF 等姿态确定与状态估计算法
- `src/control/`：控制律相关代码
- `src/disturbance/`：扰动预算与相关占位/扩展接口
- `src/utils/`：通用 MATLAB 工具函数
- `scripts/setup/`：工程初始化与路径配置脚本
- `scripts/analysis/`：分析、演示与结果检查脚本
- `tests/smoke/`：基础冒烟测试
- `docs/`：架构说明、约定文档与其他设计说明

## 快速开始

### 1. 初始化工程

在 MATLAB 中运行：

```matlab
run('scripts/setup/setup_paths.m');
init_project;
```

### 2. 查看基础姿态 EKF 演示

```matlab
run_attitude_ekf_demo;
```

### 3. 运行基础环境测试

```matlab
test_environment_models;
```

### 4. 运行姿态估计测试

```matlab
test_attitude_estimation;
```

## 当前已知边界

当前项目仍然属于“工程原型 + 结构化扩展阶段”，还不是完整任务级飞行软件。主要边界包括：

- `high_fidelity` 模式依赖 MATLAB/Aerospace Toolbox 或用户自定义高保真函数
- Simulink 顶层模型和这批 MATLAB 算法函数仍在逐步深度集成
- 一些控制、任务级场景和执行机构模型仍以骨架/占位为主
- 当前 smoke test 覆盖的是主链路基础正确性，不代表完整数值验证已完成

## 文档

- 架构说明：[docs/architecture.md](docs/architecture.md)
- 建模与命名约定：[docs/conventions.md](docs/conventions.md)
- Godot 可视化说明：[docs/godot_live_visualization.md](docs/godot_live_visualization.md)