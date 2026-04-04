function magneticMeasurement = measure_magnetometer(q_bi, B_i, sensorConfig, enableNoise)
%MEASURE_MAGNETOMETER 模拟磁力计输出的机体系地磁方向测量。
%
% 输入:
%   q_bi         : 机体系相对惯性系的四元数。
%   B_i          : 惯性系下的地磁场向量。
%   sensorConfig : 磁力计配置结构体。
%   enableNoise  : 是否叠加测量噪声，默认 true。
%
% 输出:
%   magneticMeasurement.valid    : 当前测量是否有效。
%   magneticMeasurement.vector_b : 机体系下测得的磁场向量。
%
% 说明:
%   1. 这里既可以保留磁场幅值，也可以按配置输出单位方向。
%   2. 若后续只用来做姿态估计，通常使用单位方向即可。
%   3. 若后续还要做磁控执行器建模，则可考虑保留真实幅值信息。

if nargin < 4
    enableNoise = true;
end

magneticMeasurement.valid = false;
magneticMeasurement.vector_b = zeros(3, 1);

if ~sensorConfig.enabled
    return;
end

if norm(B_i) <= sensorConfig.minSignalNorm
    return;
end

DCM_bi = q2dcm(q_bi);
BTrue_b = DCM_bi * B_i;

if enableNoise
    BTrue_b = BTrue_b + sensorConfig.noiseStdT * randn(3, 1);
end

if sensorConfig.returnUnitVector
    BTrue_b = local_unit_vector(BTrue_b);
end

magneticMeasurement.valid = all(isfinite(BTrue_b)) && norm(BTrue_b) > sensorConfig.minSignalNorm;
magneticMeasurement.vector_b = BTrue_b;
end