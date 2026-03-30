function sunVector_i = calc_sun_vector_i(t, env)
%CALC_SUN_VECTOR_I 计算惯性系下的太阳方向向量。
%
%   计算思路：
%   1. 取初始太阳方向 sunVector0_i 作为基准方向
%   2. 假设太阳在惯性系中绕 Z 轴以恒定角速度缓慢转动
%      theta = omega_sun * t
%   3. 用绕 Z 轴旋转矩阵得到当前太阳方向
%      sunVector_i = Rz(theta) * sunVector0_i
%
%   该函数输出的是方向向量，当前用于环境扰动建模，后续可替换为
%   更精确的太阳历模型。

sun0_i = local_unit_vector(env.sunVector0_i);
theta = env.sunAngularRateRadS * t;
Rz = [cos(theta), -sin(theta), 0.0; ...
      sin(theta),  cos(theta), 0.0; ...
      0.0,         0.0,        1.0];
sunVector_i = Rz * sun0_i;
end
