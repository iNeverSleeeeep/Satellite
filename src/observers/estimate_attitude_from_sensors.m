function [state, measurements] = estimate_attitude_from_sensors( ...
    t, X_i, qTrue_bi, omega_b, dt, env, sensorSuite, observerConfig, state, enableNoise)
%ESTIMATE_ATTITUDE_FROM_SENSORS 通过传感器测量和 EKF 完成姿态估计。
%
% 输入:
%   t              : 当前时刻。
%   X_i            : 卫星惯性系位置。
%   qTrue_bi       : 真实姿态，用于仿真生成传感器测量。
%   omega_b        : 机体系角速度。
%   dt             : 滤波步长。
%   env            : 环境配置。
%   sensorSuite    : 传感器配置。
%   observerConfig : EKF 配置。
%   state          : 上一时刻滤波器状态。
%   enableNoise    : 是否叠加测量噪声。
%
% 输出:
%   state          : 更新后的滤波器状态，包含姿态四元数和协方差。
%   measurements   : 当前时刻生成的太阳敏感器和磁力计测量。
%
% 处理流程:
%   1. 根据真实姿态和环境模型生成太阳敏感器/磁力计测量。
%   2. 将测量送入姿态 EKF。
%   3. 输出当前的姿态估计结果。
%
% 说明:
%   这个函数是仿真场景下的便捷封装。
%   如果你已经有真实硬件测量数据，也可以跳过本函数，直接调用
%   attitude_ekf_step，并把外部测量组装成 measurements 结构体输入。

if nargin < 10
    enableNoise = true;
end

measurements = measure_attitude_sensors(t, X_i, qTrue_bi, env, sensorSuite, enableNoise);
state = attitude_ekf_step(state, omega_b, dt, measurements, observerConfig);
end