function sunMeasurement = reconstruct_sun_vector_from_css(rawCounts, cssConfig)
%RECONSTRUCT_SUN_VECTOR_FROM_CSS 根据多个粗太阳敏感器原始测量重建太阳方向。
%
% 输入:
%   rawCounts : m x 1 原始传感器输出，可为 ADC 计数、标定前电压或已转换量。
%   cssConfig : 粗太阳敏感器阵列配置，至少包含:
%               - normals_b        : m x 3 传感器法向，表达在机体系下
%               - gain             : m x 1 增益
%               - bias             : m x 1 零偏
%               - minSignal        : 有效信号阈值
%               - minActiveSensors : 参与求解所需的最少有效传感器数量
%
% 输出:
%   sunMeasurement.valid     : 是否成功重建太阳方向。
%   sunMeasurement.vector_b  : 重建得到的机体系太阳方向单位向量。
%   sunMeasurement.usedIdx   : 参与求解的传感器编号。
%   sunMeasurement.calibrated: 标定后的各通道输出。
%
% 数学模型:
%   对每个粗太阳敏感器，标定后的输出可近似写为:
%       y_i ≈ n_i^T * sun_b
%   将全部有效传感器堆叠后可得:
%       y ≈ N * sun_b
%   其中 N 的每一行为一个传感器法向量。
%
%   当有效传感器数足够且法向分布不退化时，可用最小二乘求解 sun_b:
%       sun_b = (N^T N)^(-1) N^T y
%   然后再归一化为单位方向向量。
%
% 工程说明:
%   1. 理论上至少需要 3 个不同轴向且几何不退化的有效传感器。
%   2. 实际工程通常会布置 6 个或更多传感器，形成冗余覆盖。
%   3. 若当前被照亮的传感器太少，或者法向近似共面/退化，则无法稳定重建方向。

rawCounts = rawCounts(:);
normals_b = cssConfig.normals_b;
gain = cssConfig.gain(:);
bias = cssConfig.bias(:);
minSignal = cssConfig.minSignal;
minActiveSensors = cssConfig.minActiveSensors;

sunMeasurement.valid = false;
sunMeasurement.vector_b = zeros(3, 1);
sunMeasurement.usedIdx = [];
sunMeasurement.calibrated = zeros(size(rawCounts));

% 1. 标定修正，将原始值转换为可参与几何求解的通道响应。
calibrated = (rawCounts - bias) ./ gain;
sunMeasurement.calibrated = calibrated;

% 2. 保留被太阳照亮且信号足够强的传感器。
usedIdx = find(calibrated > minSignal);
if numel(usedIdx) < minActiveSensors
    return;
end

N = normals_b(usedIdx, :);
y = calibrated(usedIdx);

% 3. 检查几何可观性。若法向退化，则最小二乘会不稳定。
if rank(N) < 3
    return;
end

% 4. 用最小二乘重建机体系太阳方向。
sunVec_b = (N' * N) \ (N' * y);
if any(~isfinite(sunVec_b)) || norm(sunVec_b) < eps
    return;
end

sunMeasurement.valid = true;
sunMeasurement.vector_b = local_unit_vector(sunVec_b);
sunMeasurement.usedIdx = usedIdx;
end