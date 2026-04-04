function rawCounts = simulate_css_sun_sensor_raw(sunVector_b, cssConfig, enableNoise)
%SIMULATE_CSS_SUN_SENSOR_RAW 模拟粗太阳敏感器阵列的原始输出。
%
% 输入:
%   sunVector_b  : 机体系下太阳方向单位向量。
%   cssConfig    : 粗太阳敏感器阵列配置。
%                  需要包含:
%                    - normals_b : m x 3 各传感器法向
%                    - gain      : m x 1 增益
%                    - bias      : m x 1 零偏
%   enableNoise  : 是否叠加噪声。
%
% 输出:
%   rawCounts    : m x 1 各粗太阳敏感器的原始输出。
%
% 物理模型:
%   对第 i 个粗太阳敏感器，理想输出近似为:
%       s_i = gain_i * max(0, n_i^T * sun_b) + bias_i + noise_i
%   其中 max(0, ·) 表示背光时传感器不输出有效入射信号。

if nargin < 3
    enableNoise = true;
end

sunHat_b = local_unit_vector(sunVector_b);
normals_b = cssConfig.normals_b;
gain = cssConfig.gain(:);
bias = cssConfig.bias(:);

projection = max(0.0, normals_b * sunHat_b);
rawCounts = gain .* projection + bias;

if enableNoise && isfield(cssConfig, 'noiseStd')
    rawCounts = rawCounts + cssConfig.noiseStd(:) .* randn(size(rawCounts));
end
end