function q_bi = triad_attitude_init(sun_b, mag_b, sun_i, mag_i)
%TRIAD_ATTITUDE_INIT 使用 TRIAD 方法由两组方向矢量直接求解姿态。
%
% 输入:
%   sun_b : 机体系下太阳敏感器测得的太阳方向。
%   mag_b : 机体系下磁力计测得的地磁方向。
%   sun_i : 惯性系下太阳参考方向。
%   mag_i : 惯性系下地磁参考方向。
%
% 输出:
%   q_bi  : 机体系相对惯性系的姿态四元数。
%
% 核心思想:
%   TRIAD 是经典的两矢量姿态确定方法。已知两组“同一物理方向”在不同
%   坐标系下的表达，就可以分别在两个坐标系中构造一组正交基，然后求得
%   两组基之间的旋转矩阵，最终得到姿态。
%
% 具体步骤:
%   1. 在机体系中，以太阳方向为第一基向量。
%   2. 利用太阳方向与磁场方向叉乘，构造第二基向量。
%   3. 再由前两项叉乘得到第三基向量，形成机体系 TRIAD。
%   4. 在惯性系中重复上述过程，形成惯性系 TRIAD。
%   5. 由 DCM_bi = T_b * T_i' 得到姿态矩阵，再转为四元数。
%
% 注意:
%   若太阳方向与磁场方向近似共线，则两矢量法退化，此时无法稳定确定姿态。

sun_b = local_unit_vector(sun_b);
mag_b = local_unit_vector(mag_b);
sun_i = local_unit_vector(sun_i);
mag_i = local_unit_vector(mag_i);

if norm(cross(sun_b, mag_b)) < 1e-8 || norm(cross(sun_i, mag_i)) < 1e-8
    q_bi = [1.0; 0.0; 0.0; 0.0];
    return;
end

t1_b = sun_b;
t2_b = local_unit_vector(cross(sun_b, mag_b));
t3_b = cross(t1_b, t2_b);

t1_i = sun_i;
t2_i = local_unit_vector(cross(sun_i, mag_i));
t3_i = cross(t1_i, t2_i);

DCM_bi = [t1_b, t2_b, t3_b] * [t1_i, t2_i, t3_i]';
q_bi = dcm2q(DCM_bi);
end