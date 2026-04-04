function magneticMeasurement = calibrate_magnetometer_raw(rawVector_b, calibrationConfig, returnUnitVector, minSignalNorm)
%CALIBRATE_MAGNETOMETER_RAW 根据真实三轴磁力计原始读数恢复机体系磁场向量。
%
% 功能概述:
%   本函数用于把磁力计的“原始三轴输出”转换为“可用于姿态估计的机体系磁场向量”。
%   真实磁力计采集到的原始值通常不能直接拿来做姿态估计，因为其中会混入:
%   1. 零偏 / 硬铁偏置
%   2. 各轴灵敏度不一致
%   3. 轴间耦合
%   4. 软磁畸变
%   5. 传感器安装不正交或安装误差
%
%
% 输入参数:
%   rawVector_b       : 3x1 原始磁力计三轴读数。
%                       可以是 ADC 转换后的物理量，也可以是已经按量程缩放后的磁场读数。
%   calibrationConfig : 标定参数结构体，至少包含:
%                       - bias           : 3x1 零偏/硬铁偏置
%                       - scale          : 3x1 各轴比例因子
%                       - softIronMatrix : 3x3 软磁/安装矩阵校正项
%   returnUnitVector  : 是否输出单位方向向量。
%                       若为 true，则输出仅保留方向信息。
%                       若为 false，则输出保留幅值信息。
%   minSignalNorm     : 有效信号阈值，用于排除接近零的无意义数据。
%
% 输出参数:
%   magneticMeasurement.valid        : 标定结果是否有效。
%   magneticMeasurement.vector_b     : 最终输出的机体系磁场向量。
%   magneticMeasurement.calibrated_b : 标定后但未必单位化的机体系磁场向量。
%
% 标定模型:
%   当前采用如下简化工程模型:
%       compensated   = (raw - bias) ./ scale
%       B_calibrated  = softIronMatrix * compensated
%
%   其中:
%   1. raw - bias
%      用于去除零偏。这里的 bias 既可以表示电子零漂，也可以近似表示硬铁偏置。
%
%   2. ./ scale
%      用于补偿各轴增益不一致。例如 X/Y/Z 三轴对同样大小的磁场响应不同。
%
%   3. softIronMatrix * compensated
%      用于补偿更一般的线性畸变，包括:
%      - 软磁效应导致的椭球畸变
%      - 轴间耦合
%      - 传感器安装不正交
%      - 坐标轴轻微错位
%
% 原理解释:
%   理想情况下，磁力计在不同姿态下测得的磁场点云应落在一个球面上。
%   但真实传感器常常因为偏置、比例因子失配和软磁效应，使点云变成“偏移的椭球”。
%   标定的目标，就是把这个偏移椭球重新映射回球面。
%
%   从几何上理解:
%   1. bias 负责把椭球中心移回原点。
%   2. scale 负责把各轴拉伸量调回一致。
%   3. softIronMatrix 负责处理旋转、耦合和一般线性畸变。
%
%   因此 softIronMatrix 的作用不是“单独从零推断一个向量”，而是对已经去偏置、
%   去比例因子的三轴测量做线性校正，使它更接近真实磁场向量。
%
% 工程背景:
%   在磁力计标定中，经常把误差分为两类:
%   1. Hard-Iron Error
%      主要体现为一个恒定偏置，使整个测量球面整体平移。
%   2. Soft-Iron Error
%      主要体现为方向相关的线性畸变，使球面被压缩、拉伸、旋转成椭球。
%
%   本函数中的命名沿用了工程上常见说法，把矩阵项称为 softIronMatrix。
%   严格来说，它也可能同时吸收了一部分安装矩阵和轴不正交误差，不一定只包含
%   “狭义软磁效应”。
%
% 使用建议:
%   1. 如果后续只做姿态估计，通常可以设置 returnUnitVector = true，
%      因为 EKF 更关心磁场方向而不是幅值。
%   2. 如果后续还要做磁场强度分析或磁控执行器相关建模，则可保留幅值信息。
%   3. 若你的真实标定结果更复杂，也可以把 scale 和 softIronMatrix 合并为一个
%      更一般的 3x3 标定矩阵。

if nargin < 3 || isempty(returnUnitVector)
    returnUnitVector = true;
end

if nargin < 4 || isempty(minSignalNorm)
    minSignalNorm = 1e-12;
end

rawVector_b = rawVector_b(:);
bias = calibrationConfig.bias(:);
scale = calibrationConfig.scale(:);
softIronMatrix = calibrationConfig.softIronMatrix;

magneticMeasurement.valid = false;
magneticMeasurement.vector_b = zeros(3, 1);
magneticMeasurement.calibrated_b = zeros(3, 1);

% 第一步: 去除三轴零偏/硬铁偏置。
% 这一步的结果仍然可能带有各轴增益失配和轴间耦合误差。
compensated = (rawVector_b - bias) ./ scale;

% 第二步: 施加矩阵校正，补偿软磁畸变、安装矩阵误差和轴间耦合。
calibrated_b = softIronMatrix * compensated;
magneticMeasurement.calibrated_b = calibrated_b;

% 若结果非有限或幅值几乎为零，则认为当前结果无效。
if any(~isfinite(calibrated_b)) || norm(calibrated_b) <= minSignalNorm
    return;
end

% 若姿态估计只关心方向，则将标定后的向量单位化。
if returnUnitVector
    magneticVector_b = local_unit_vector(calibrated_b);
else
    magneticVector_b = calibrated_b;
end

magneticMeasurement.valid = all(isfinite(magneticVector_b)) && norm(magneticVector_b) > minSignalNorm;
magneticMeasurement.vector_b = magneticVector_b;
end