function force = calc_gravity_b(X_i, Mass, DCM_bi)
%计算含 J2 摄动的重力，并输出体系力矢量。
%   force = calc_gravity_b(X_i, Mass, DCM_bi)
%
%   输入：
%   X_i : 惯性系下卫星位置向量 [x; y; z]，单位 m
%   Mass   : 卫星质量，单位 kg
%   DCM_bi : 从惯性系到体系的方向余弦矩阵
%
%   输出：
%   force : 体系下重力矢量 [fx; fy; fz]，单位 N
%
%   说明：
%   1. 本函数先在惯性系下计算中心引力和 J2 摄动加速度。
%   2. 再通过 DCM_bi 将惯性系重力变换到体系。
%   3. 这里默认惯性系 Z 轴与地球自转轴对齐，否则 J2 项不再严格成立。

%% 1. 常数（WGS-84）
mu       = 3.986004418e14;    % 地球引力常数，m^3/s^2
R_eq     = 6378137.0;         % 地球赤道半径，m
J2       = 1.082635854e-3;    % J2 二阶带谐系数

%% 2. 位置分量与距离量
x = X_i(1);
y = X_i(2);
z = X_i(3);

r2 = x^2 + y^2 + z^2;
r  = sqrt(r2);
r3 = r^3;
r5 = r^5;

%% 3. 中心引力项
a_x0 = -mu * x / r3;
a_y0 = -mu * y / r3;
a_z0 = -mu * z / r3;

%% 4. J2 摄动加速度
factor = 1.5 * J2 * mu * R_eq^2 / r5;

a_x_J2 = factor * x * (5 * z^2 / r2 - 1);
a_y_J2 = factor * y * (5 * z^2 / r2 - 1);
a_z_J2 = factor * z * (5 * z^2 / r2 - 3);

%% 5. 惯性系下总重力加速度
a = [a_x0 + a_x_J2;
    a_y0 + a_y_J2;
    a_z0 + a_z_J2];

%% 6. 转换到体系并乘以质量，得到重力
force = DCM_bi * (Mass * a);
end
