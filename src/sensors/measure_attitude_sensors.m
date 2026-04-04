function measurements = measure_attitude_sensors(t, X_i, q_bi, env, sensorSuite, enableNoise)
%MEASURE_ATTITUDE_SENSORS 统一生成姿态估计所需的传感器测量。
%
% 输入:
%   t           : 当前仿真时刻。
%   X_i         : 卫星在惯性系下的位置向量。
%   q_bi        : 当前真实姿态四元数。
%   env         : 环境模型配置。
%   sensorSuite : 传感器配置集合。
%   enableNoise : 是否叠加传感器噪声。
%
% 输出:
%   measurements.reference.sun_i : 惯性系参考太阳方向。
%   measurements.reference.mag_i : 惯性系参考地磁方向。
%   measurements.sun             : 太阳敏感器测量结果。
%   measurements.mag             : 磁力计测量结果。
%
% 作用:
%   本函数把“环境参考矢量生成”和“传感器测量生成”打包起来，便于
%   后续观测器直接使用。对于姿态确定问题，我们需要两组信息:
%   1. 惯性系参考方向
%   2. 机体系测量方向
%   当太阳方向和地磁方向都可用时，即可使用两矢量姿态确定方法计算姿态。

if nargin < 6
    enableNoise = true;
end

sunVector_i = calc_sun_vector_i(t, env);
magneticField_i = calc_magnetic_field_i(X_i, env, t);

measurements.t = t;
measurements.reference.sun_i = local_unit_vector(sunVector_i);
measurements.reference.mag_i = local_unit_vector(magneticField_i);
measurements.sun = measure_sun_sensor(q_bi, sunVector_i, sensorSuite.sunSensor, enableNoise);
measurements.mag = measure_magnetometer(q_bi, magneticField_i, sensorSuite.magnetometer, enableNoise);
end