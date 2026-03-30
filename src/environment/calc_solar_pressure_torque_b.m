function torque_b = calc_solar_pressure_torque_b(sunVector_i, DCM_bi, spacecraft, env)
%CALC_SOLAR_PRESSURE_TORQUE_B 计算星体系下的太阳光压力矩。
%
%   计算思路：
%   1. 先调用太阳光压模型，得到星体系下的太阳光压力 force_b
%   2. 计算压心相对质心的力臂
%      r_cp = centerOfPressureM - centerOfMassM
%   3. 由力矩定义计算
%      T_srp = r_cp x F_srp
%
%   输出 torque_b 为星体系下的太阳光压力矩，单位 N*m。

force_b = calc_solar_radiation_pressure_b(sunVector_i, DCM_bi, spacecraft, env);
leverArm_b = spacecraft.centerOfPressureM - spacecraft.centerOfMassM;
torque_b = cross(leverArm_b, force_b);
end
