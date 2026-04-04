function cfg = attitude_ekf_config()
%ATTITUDE_EKF_CONFIG 默认姿态扩展卡尔曼滤波器配置。
%
% 状态定义:
%   当前实现使用四元数 q_bi 作为滤波状态。
%
% 参数说明:
%   initialCovariance    : 初始状态协方差。
%   processNoise         : 过程噪声协方差，用于描述姿态传播模型的不确定性。
%   sunMeasurementNoise  : 太阳敏感器量测噪声协方差。
%   magMeasurementNoise  : 磁力计量测噪声协方差。
%   finiteDifferenceStep : 数值雅可比计算时的微小扰动步长。
%   initialQuaternion    : 无可靠先验时的默认初始姿态。
%
% 备注:
%   当前版本更偏向工程原型验证，优先保证结构清晰和可读性。
%   如果后续需要更高精度或更强实时性，可以进一步改为误差状态 EKF。

cfg.initialCovariance = 1e-2 * eye(4);
cfg.processNoise = 1e-6 * eye(4);
cfg.sunMeasurementNoise = (5e-3 ^ 2) * eye(3);
cfg.magMeasurementNoise = (1e-2 ^ 2) * eye(3);
cfg.finiteDifferenceStep = 1e-6;
cfg.initialQuaternion = [1.0; 0.0; 0.0; 0.0];
end