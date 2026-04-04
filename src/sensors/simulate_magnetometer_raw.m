function rawVector_b = simulate_magnetometer_raw(BTrue_b, sensorConfig, enableNoise)
%SIMULATE_MAGNETOMETER_RAW 模拟真实三轴磁力计的原始输出。
%
% 功能概述:
%   本函数用于从“真实机体系磁场向量”生成“更接近真实硬件输出的三轴原始读数”。
%   它主要用于仿真和算法联调，帮助验证磁力计标定链路是否正确。
%
%   与 calibrate_magnetometer_raw 的关系是:
%   1. 本函数负责“加上误差”，得到原始测量值。
%   2. calibrate_magnetometer_raw 负责“去掉误差”，恢复磁场向量。
%   两者在当前简化模型下近似互逆。
%
% 输入参数:
%   BTrue_b      : 3x1 机体系下真实磁场向量。
%                  这是理想情况下希望磁力计测到的真实值。
%   sensorConfig : 磁力计配置结构体，内部需要包含:
%                  - calibration.bias
%                  - calibration.scale
%                  - calibration.softIronMatrix
%                  - noiseStdT
%   enableNoise  : 是否叠加测量噪声。
%
% 输出参数:
%   rawVector_b  : 3x1 三轴磁力计原始读数。
%
% 仿真模型:
%   当前采用简化线性模型构造原始测量值:
%       raw = scale .* (softIronMatrix \ BTrue_b) + bias + noise
%
% 这个公式是为了与标定公式配对:
%       B_calibrated = softIronMatrix * ((raw - bias) ./ scale)
%
% 因此，在忽略噪声的情况下，有:
%       calibrate_magnetometer_raw(simulate_magnetometer_raw(BTrue_b)) ≈ BTrue_b
%
% 原理解释:
%   真实磁力计的原始输出往往不是“真实磁场向量本身”，而是带有多种误差后的结果。
%   常见误差包括:
%   1. 零偏 / 硬铁偏置
%   2. 各轴比例因子不一致
%   3. 软磁畸变
%   4. 轴间耦合
%   5. 安装不正交
%
%   在这个仿真模型中:
%   1. bias 用于给每个轴加入固定偏置
%   2. scale 用于让不同轴具有不同增益
%   3. softIronMatrix 的逆作用在真实磁场上，用于制造椭球畸变和轴耦合
%
% 为什么这里要用 softIronMatrix \ BTrue_b:
%   因为标定函数中会乘以 softIronMatrix 去做恢复，为了让“仿真误差模型”和
%   “标定恢复模型”尽量互相匹配，这里用左除近似表示先施加一个相反方向的线性畸变。
%
% 工程理解:
%   你可以把这个过程看成:
%   1. 先把理想球面数据扭成椭球
%   2. 再对各轴加偏置和缩放
%   3. 最后叠加测量噪声
%   这样得到的 rawVector_b 就更接近真实磁力计会输出的数据。
%
% 使用建议:
%   1. 做 EKF 闭环联调时，可先 enableNoise = false，验证标定链是否正确。
%   2. 验证通过后，再逐步加噪声、偏置和软磁矩阵，观察姿态估计鲁棒性。
%   3. 若后续有更真实的传感器模型，也可以在本函数基础上继续扩展。

if nargin < 3
    enableNoise = true;
end

bias = sensorConfig.calibration.bias(:);
scale = sensorConfig.calibration.scale(:);
softIronMatrix = sensorConfig.calibration.softIronMatrix;

% 第一步: 对真实磁场施加与标定相反方向的线性畸变，构造“未校正”的理想原始值。
rawVector_b = scale .* (softIronMatrix \ BTrue_b) + bias;

% 第二步: 叠加测量噪声，模拟真实传感器电子噪声和量化误差的综合影响。
if enableNoise
    rawVector_b = rawVector_b + sensorConfig.noiseStdT * randn(3, 1);
end
end