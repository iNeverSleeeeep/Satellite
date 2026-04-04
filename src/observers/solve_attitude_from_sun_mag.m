function q_bi = solve_attitude_from_sun_mag(sun_b, mag_b, sun_i, mag_i)
%SOLVE_ATTITUDE_FROM_SUN_MAG 根据太阳敏感器和磁力计方向直接计算姿态。
%
% 这是“由太阳敏感器方向 + 磁力计方向直接求姿态”的明确入口文件。
% 若你在工程中寻找“两矢量直接定姿”的实现，优先看这个函数。
%
% 输入:
%   sun_b : 机体系下太阳敏感器测得的太阳方向。
%   mag_b : 机体系下磁力计测得的磁场方向。
%   sun_i : 惯性系下太阳参考方向。
%   mag_i : 惯性系下地磁参考方向。
%
% 输出:
%   q_bi  : 求得的姿态四元数，表示机体系相对惯性系的旋转。
%
% 说明:
%   当前实现内部调用 TRIAD 方法完成姿态确定。
%   在 EKF 中，这个直接定姿结果主要用于:
%   1. 作为滤波器初始化姿态
%   2. 在需要时作为无陀螺快速定姿的独立结果

q_bi = triad_attitude_init(sun_b, mag_b, sun_i, mag_i);
end