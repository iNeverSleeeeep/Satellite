function B_i = calc_magnetic_field_i(X_i, env)
%CALC_MAGNETIC_FIELD_I 计算惯性系下的地磁场向量。
%
%   计算公式：
%   B = mu0 / (4*pi*r^3) * (3 * r_hat * (m dot r_hat) - m)
%   其中 m 为地球磁偶极矩方向向量，r_hat 为卫星位置方向单位向量。
%
%   计算思路：
%   1. 使用地心偶极磁场作为简化地磁模型
%   2. 假设地球磁偶极轴在惯性系中固定不变
%   3. 输出 B_i 为惯性系下的磁场向量，单位 T

mu0 = 4.0 * pi * 1e-7;
r = norm(X_i);

if r < eps
    B_i = zeros(3, 1);
    return;
end

r_hat_i = X_i / r;
m_hat_i = local_unit_vector(env.earthDipoleAxis_i);
m_i = env.earthMagneticDipoleT * m_hat_i;
B_i = mu0 / (4.0 * pi * r^3) * (3.0 * r_hat_i * dot(m_i, r_hat_i) - m_i);
end
