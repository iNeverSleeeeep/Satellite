function torque_b = calc_gravity_gradient_torque_b(X_i, DCM_bi, inertiaKgM2, env)
%CALC_GRAVITY_GRADIENT_TORQUE_B 计算星体系下的重力梯度力矩。
%
%   计算公式：
%   T_gg = 3 * mu / r^3 * (r_hat x (J * r_hat))
%   其中 mu 为地球引力常数，r_hat 为航天器指向地心方向在星体系下的
%   单位向量，J 为航天器相对质心的转动惯量矩阵。
%
%   计算步骤：
%   1. 将位置向量 X_i 从惯性系转换到星体系，得到 r_b
%   2. 归一化得到 r_hat_b
%   3. 按上式计算重力梯度力矩，输出 torque_b，单位 N*m

r_b = DCM_bi * X_i;
r_hat_b = local_unit_vector(r_b);
r_norm = norm(X_i);

if r_norm < eps
    torque_b = zeros(3, 1);
    return;
end

torque_b = 3 * env.earthMuM3S2 / r_norm^3 * cross(r_hat_b, inertiaKgM2 * r_hat_b);
end
