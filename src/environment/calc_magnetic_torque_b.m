function torque_b = calc_magnetic_torque_b(B_i, DCM_bi, dipole_b)
%CALC_MAGNETIC_TORQUE_B 计算星体系下的磁力矩。
%
%   计算公式：
%   T_m = m x B
%   其中 m 为星体系下的等效磁偶极矩，B 为星体系下的地磁场向量。
%
%   计算步骤：
%   1. 将惯性系下的磁场向量 B_i 转换到星体系，得到 B_b
%   2. 计算磁偶极矩与磁场的叉乘，得到磁力矩 torque_b，单位 N*m

B_b = DCM_bi * B_i;
torque_b = cross(dipole_b, B_b);
end
