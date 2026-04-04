function sensorConfig = default_sensor_config()
%DEFAULT_SENSOR_CONFIG 默认姿态传感器配置。
%
% 该配置文件用于集中管理姿态测量链中的传感器参数，当前包含:
%   1. 太阳敏感器
%   2. 磁力计
%
% 约定:
%   1. 若 returnUnitVector = true，则输出被归一化为单位方向向量。
%   2. 噪声参数采用标准差形式给出。
%   3. minSignalNorm 用于防止在信号接近零时继续输出无意义测量。
%   4. 对于真实粗太阳敏感器阵列，可使用 css 子配置从原始测量值反算太阳方向。
%   5. 对于真实磁力计，可使用 calibration 子配置从原始三轴读数恢复磁场向量。

sensorConfig.sampleTime = 0.1;

sensorConfig.sunSensor.enabled = true;
sensorConfig.sunSensor.mode = 'ideal-vector';
sensorConfig.sunSensor.noiseStd = 5e-3;
sensorConfig.sunSensor.minSignalNorm = 1e-9;
sensorConfig.sunSensor.returnUnitVector = true;

% 粗太阳敏感器阵列的默认配置。
% 这里采用 6 面体安装方式，法向分别指向机体系的 +/-X、+/-Y、+/-Z。
sensorConfig.sunSensor.css.normals_b = [ ...
     1.0,  0.0,  0.0; ...
    -1.0,  0.0,  0.0; ...
     0.0,  1.0,  0.0; ...
     0.0, -1.0,  0.0; ...
     0.0,  0.0,  1.0; ...
     0.0,  0.0, -1.0];
sensorConfig.sunSensor.css.bias = zeros(6, 1);
sensorConfig.sunSensor.css.gain = ones(6, 1);
sensorConfig.sunSensor.css.minSignal = 0.02;
sensorConfig.sunSensor.css.minActiveSensors = 3;

sensorConfig.magnetometer.enabled = true;
sensorConfig.magnetometer.mode = 'ideal-vector';
sensorConfig.magnetometer.noiseStdT = 2e-7;
sensorConfig.magnetometer.minSignalNorm = 1e-12;
sensorConfig.magnetometer.returnUnitVector = true;

% 磁力计标定参数。
% raw_b 为原始三轴读数时，采用如下模型进行修正:
%   B_calibrated = softIronMatrix * ((raw_b - bias) ./ scale)
sensorConfig.magnetometer.calibration.bias = zeros(3, 1);
sensorConfig.magnetometer.calibration.scale = ones(3, 1);
sensorConfig.magnetometer.calibration.softIronMatrix = eye(3);
end