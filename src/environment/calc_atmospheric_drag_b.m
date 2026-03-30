function force_b = calc_atmospheric_drag_b(X_i, V_i, DCM_bi, spacecraft, env)
%CALC_ATMOSPHERIC_DRAG_B 计算星体系下的大气阻力。
%
%   force_b = calc_atmospheric_drag_b(X_i, V_i, DCM_bi, spacecraft, env)
%   计算思路：
%   1. 先根据轨道高度用指数大气模型估算密度 rho
%      rho = rho0 * exp(-(h - h0) / H)
%   2. 计算航天器相对大气的速度，默认大气随地球自转
%      v_rel_i = V_i - omega_ie x X_i
%   3. 将相对速度转换到星体系，阻力方向与相对速度方向相反
%   4. 使用阻力公式
%      F_d = -1/2 * rho * Cd * A * |v_rel|^2 * e_v
%      其中 e_v 为相对速度方向的单位向量
%
%   输入输出：
%   X_i、V_i 在惯性系下表示，DCM_bi 用于将惯性系向量转换到星体系，
%   输出 force_b 为星体系下的三维阻力向量，单位 N。

rho = calc_atmospheric_density(X_i, env);

omega_ie_i = [0.0; 0.0; env.earthRotationRadS];
v_rel_i = V_i - cross(omega_ie_i, X_i);
v_rel_b = DCM_bi * v_rel_i;
speed = norm(v_rel_b);

if speed < eps
    force_b = zeros(3, 1);
    return;
end

dragDir_b = -v_rel_b / speed;
force_b = 0.5 * rho * speed^2 * spacecraft.dragCoefficient ...
    * spacecraft.dragAreaM2 * dragDir_b;
end
