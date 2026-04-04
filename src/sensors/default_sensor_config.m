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

sensorConfig.sampleTime = 0.1;

sensorConfig.sunSensor.enabled = true;
sensorConfig.sunSensor.noiseStd = 5e-3;
sensorConfig.sunSensor.minSignalNorm = 1e-9;
sensorConfig.sunSensor.returnUnitVector = true;

sensorConfig.magnetometer.enabled = true;
sensorConfig.magnetometer.noiseStdT = 2e-7;
sensorConfig.magnetometer.minSignalNorm = 1e-12;
sensorConfig.magnetometer.returnUnitVector = true;
end