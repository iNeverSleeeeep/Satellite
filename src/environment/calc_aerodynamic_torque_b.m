function torque_b = calc_aerodynamic_torque_b(X_i, V_i, DCM_bi, spacecraft, env)
%CALC_AERODYNAMIC_TORQUE_B 计算星体系下的阻力矩。
%
%   计算思路：
%   1. 先调用大气阻力模型，得到星体系下的阻力 force_b
%   2. 计算气动作用点相对质心的力臂
%      r_cd = centerOfDragM - centerOfMassM
%   3. 由力矩定义计算
%      T_drag = r_cd x F_drag
%
%   输出 torque_b 为星体系下的阻力矩，单位 N*m。

force_b = calc_atmospheric_drag_b(X_i, V_i, DCM_bi, spacecraft, env);
leverArm_b = spacecraft.centerOfDragM - spacecraft.centerOfMassM;
torque_b = cross(leverArm_b, force_b);
end
