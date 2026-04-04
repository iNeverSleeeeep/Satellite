function sunMeasurement = measure_sun_sensor(q_bi, sunVector_i, sensorConfig, enableNoise)
%MEASURE_SUN_SENSOR 模拟太阳敏感器输出的机体系太阳方向测量。
%
% 输入:
%   q_bi         : 机体系相对惯性系的四元数。
%   sunVector_i  : 惯性系下太阳方向向量。
%   sensorConfig : 太阳敏感器配置结构体。
%   enableNoise  : 是否叠加测量噪声，默认 true。
%
% 输出:
%   sunMeasurement.valid    : 当前测量是否有效。
%   sunMeasurement.vector_b : 机体系下测得的太阳方向。
%
% 算法流程:
%   1. 检查太阳敏感器是否启用。
%   2. 检查参考太阳向量是否足够大，避免零向量参与运算。
%   3. 利用姿态四元数将惯性系太阳方向转换到机体系。
%   4. 根据配置决定是否叠加高斯噪声。
%   5. 若要求输出单位方向，则再做一次归一化。

if nargin < 4
    enableNoise = true;
end

sunMeasurement.valid = false;
sunMeasurement.vector_b = zeros(3, 1);

if ~sensorConfig.enabled
    return;
end

if norm(sunVector_i) <= sensorConfig.minSignalNorm
    return;
end

DCM_bi = q2dcm(q_bi);
sunTrue_b = DCM_bi * local_unit_vector(sunVector_i);

if enableNoise
    sunTrue_b = sunTrue_b + sensorConfig.noiseStd * randn(3, 1);
end

if sensorConfig.returnUnitVector
    sunTrue_b = local_unit_vector(sunTrue_b);
end

sunMeasurement.valid = all(isfinite(sunTrue_b)) && norm(sunTrue_b) > sensorConfig.minSignalNorm;
sunMeasurement.vector_b = sunTrue_b;
end