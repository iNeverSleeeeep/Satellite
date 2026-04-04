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
%   magneticMeasurement.valid       : 当前测量是否有效。
%   magneticMeasurement.vector_b    : 机体系下测得的磁场向量。
%   magneticMeasurement.rawVector_b : 若为仿真原始模式，则记录原始三轴输出。
%
% 说明:
%   1. 默认模式为 'ideal-vector'，即直接使用真实机体系磁场生成测量。
%   2. 若 mode = 'raw-3axis'，则先模拟三轴磁力计原始读数，再调用
%      calibrate_magnetometer_raw 进行标定恢复。
%   3. 在真实硬件场景下，通常应直接把真实采样值传给:
%         calibrate_magnetometer_raw(rawVector_b, sensorConfig.calibration, ...)
%      而不是使用本函数。

if nargin < 4
    enableNoise = true;
end

magneticMeasurement.valid = false;
magneticMeasurement.vector_b = zeros(3, 1);
magneticMeasurement.rawVector_b = [];

if ~sensorConfig.enabled
    return;
end

if norm(B_i) <= sensorConfig.minSignalNorm
    return;
end

DCM_bi = q2dcm(q_bi);
BTrue_b = DCM_bi * B_i;
mode = local_get_mode(sensorConfig);

switch mode
    case 'ideal-vector'
        if enableNoise
            BTrue_b = BTrue_b + sensorConfig.noiseStdT * randn(3, 1);
        end

        if sensorConfig.returnUnitVector
            BTrue_b = local_unit_vector(BTrue_b);
        end

        magneticMeasurement.valid = all(isfinite(BTrue_b)) && norm(BTrue_b) > sensorConfig.minSignalNorm;
        magneticMeasurement.vector_b = BTrue_b;

    case 'raw-3axis'
        rawVector_b = simulate_magnetometer_raw(BTrue_b, sensorConfig, enableNoise);
        magneticMeasurement = calibrate_magnetometer_raw( ...
            rawVector_b, sensorConfig.calibration, sensorConfig.returnUnitVector, sensorConfig.minSignalNorm);
        magneticMeasurement.rawVector_b = rawVector_b;

    otherwise
        error('measure_magnetometer:UnsupportedMode', ...
            'Unsupported magnetometer mode: %s', mode);
end
end

function mode = local_get_mode(sensorConfig)
%LOCAL_GET_MODE 获取磁力计模式，兼容旧配置。
if isfield(sensorConfig, 'mode') && ~isempty(sensorConfig.mode)
    mode = sensorConfig.mode;
else
    mode = 'ideal-vector';
end
end