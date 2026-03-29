# 卫星仿真项目

本仓库按工程化的 MATLAB/Simulink 卫星仿真项目方式组织，方便后续继续扩展动力学、环境、GNC、控制与分析流程。

## 目录结构

- `models/plant/`：卫星本体动力学与被控对象模型
- `models/mission/`：任务级场景模型、顶层仿真装配与工况变体
- `models/libraries/`：可复用的 Simulink 模块库和公共子系统
- `src/parameters/`：卫星、轨道、传感器、执行机构等参数定义
- `src/environment/`：重力、气动、磁场、太阳辐射压等环境模型
- `src/gnc/`：制导、导航、估计与姿态确定相关算法
- `src/control/`：姿态控制、轨道控制等控制律实现
- `src/disturbance/`：扰动力与扰动力矩模型
- `src/utils/`：通用 MATLAB 工具函数
- `scripts/setup/`：工程初始化与路径配置脚本
- `scripts/analysis/`：仿真后处理、绘图和结果分析脚本
- `config/`：仿真配置、求解器配置和场景模板
- `data/input/`：外部输入数据，如星历、参考轨迹和常量表
- `data/reference/`：基准数据、对比数据和验证参考
- `results/figures/`：导出的图像结果
- `results/logs/`：仿真日志与输出数据
- `tests/smoke/`：基础冒烟测试和最小回归检查
- `docs/`：设计说明、接口说明和建模约定

## 当前模型

- `models/plant/satellite.slx`
- `models/plant/Spacecraft_Dynamics.slx`

## 建议使用方式

1. 打开 MATLAB 后先运行 `scripts/setup/setup_paths.m`
2. 将可调参数尽量集中放在 `src/parameters/`
3. 将不同任务场景和工况变体放在 `models/mission/`
4. 将仿真生成结果统一输出到 `results/`，避免根目录堆积临时文件

## 命名约定

为避免坐标系、姿态方向和变量含义混淆，建议在本项目中统一采用以下命名规则。

### 坐标系缩写

- `i`：惯性系
- `b`：星体系
- `f`：地固系

### 方向余弦矩阵

本项目推荐采用“目标坐标系在前，原坐标系在后”的命名方式：

- `DCM_bi`：将惯性系向量转换到体系
- `DCM_ib`：将体系向量转换到惯性系
- `DCM_if`：将地固系向量转换到惯性系
- `DCM_bf`：将地固系向量转换到体系

对应关系示例：

```matlab
v_b = DCM_bi * v_i;
v_i = DCM_ib * v_b;
v_b = DCM_bf * v_f;
```

### 四元数

四元数建议和方向余弦矩阵保持同一方向定义：

- `q_bi`：表示从惯性系到体系的姿态四元数
- `q_bi_0` 或 `q_b0`：初始姿态四元数

如果 `q2dcm(q_bi)` 输出的是惯性系到体系的矩阵，则可直接得到 `DCM_bi`。

### 角速度与角加速度

角速度建议默认表示“星体系相对惯性系的角速度，且分量在体系表达”：

- `omega_b`：体系角速度
- `omega_dot_b`：体系角加速度
- `omega_b0`：初始体系角速度

如果后续模型中会同时出现多种相对角速度，可使用更完整的写法，例如 `omega_bi_b`，但不建议在当前阶段将常用变量命名得过长。

### 平动状态

平动主状态如果在惯性系积分，建议统一命名为：

- `X_i`：惯性系位置
- `V_i`：惯性系速度
- `A_i`：惯性系加速度

如果需要同时输出地固系状态，可写为：

- `X_f`
- `V_f`
- `A_f`
