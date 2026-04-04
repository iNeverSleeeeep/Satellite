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
%   sunMeasurement.valid       : 当前测量是否有效。
%   sunMeasurement.vector_b    : 机体系下测得的太阳方向。
%   sunMeasurement.rawCounts   : 若模拟粗太阳敏感器阵列，则给出各探头原始输出。
%   sunMeasurement.usedIdx     : 若进行了阵列重建，则记录参与求解的传感器编号。
%
% 说明:
%   当前函数主要用于仿真场景。默认模式为 'ideal-vector'，即直接由真实姿态
%   生成机体系太阳方向，并可叠加噪声。
%
%   如果将 sensorConfig.mode 设为 'css-array'，则函数会先模拟多个粗太阳敏感器
%   的原始输出，再调用 reconstruct_sun_vector_from_css 根据原始值重建太阳方向。
%
%   对于真实硬件场景，通常不应再调用本函数，而是直接使用:
%       reconstruct_sun_vector_from_css(rawCounts, sensorConfig.css)
%   其中 rawCounts 来自真实传感器采样值。

if nargin < 4
    enableNoise = true;
end

sunMeasurement.valid = false;
sunMeasurement.vector_b = zeros(3, 1);
sunMeasurement.rawCounts = [];
sunMeasurement.usedIdx = [];

if ~sensorConfig.enabled
    return;
end

if norm(sunVector_i) <= sensorConfig.minSignalNorm
    return;
end

DCM_bi = q2dcm(q_bi);
sunTrue_b = DCM_bi * local_unit_vector(sunVector_i);
mode = local_get_mode(sensorConfig);

switch mode
    case 'ideal-vector'
        if enableNoise
            sunTrue_b = sunTrue_b + sensorConfig.noiseStd * randn(3, 1);
        end

        if sensorConfig.returnUnitVector
            sunTrue_b = local_unit_vector(sunTrue_b);
        end

        sunMeasurement.valid = all(isfinite(sunTrue_b)) && norm(sunTrue_b) > sensorConfig.minSignalNorm;
        sunMeasurement.vector_b = sunTrue_b;

    case 'css-array'
        rawCounts = simulate_css_sun_sensor_raw(sunTrue_b, sensorConfig.css, enableNoise);
        sunMeasurement = reconstruct_sun_vector_from_css(rawCounts, sensorConfig.css);
        sunMeasurement.rawCounts = rawCounts;

    otherwise
        error('measure_sun_sensor:UnsupportedMode', ...
            'Unsupported sun sensor mode: %s', mode);
end
end

function mode = local_get_mode(sensorConfig)
%LOCAL_GET_MODE 获取太阳敏感器模式，兼容旧配置。
if isfield(sensorConfig, 'mode') && ~isempty(sensorConfig.mode)
    mode = sensorConfig.mode;
else
    mode = 'ideal-vector';
end
end