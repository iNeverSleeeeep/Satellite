function force_b = calc_solar_radiation_pressure_b(sunVector_i, DCM_bi, spacecraft, env)
%CALC_SOLAR_RADIATION_PRESSURE_B 计算星体系下的太阳光压力。
%
%   sunVector_i 表示从航天器指向太阳的惯性系向量，输出力在星体系下表示。
%   计算思路：
%   1. 将太阳方向向量归一化，得到受光方向单位向量
%   2. 将其转换到星体系
%   3. 按简化太阳光压模型计算
%      F_srp = -P_s * C_r * A_s * s_hat
%      其中 P_s 为太阳辐射压，C_r 为反射系数，A_s 为受光面积，
%      s_hat 为指向太阳的单位向量
%
%   这里负号表示太阳光压力方向与“指向太阳”的向量方向相反。

sunDir_i = local_unit_vector(sunVector_i);
sunDir_b = DCM_bi * sunDir_i;

force_b = -env.solarPressureNPM2 * spacecraft.solarPressureCoefficient ...
    * spacecraft.solarAreaM2 * sunDir_b;
end
