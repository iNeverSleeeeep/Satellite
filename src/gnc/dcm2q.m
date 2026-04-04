function q = dcm2q(DCM)
%DCM2Q 将方向余弦矩阵转换为标量在前的四元数。
%
% 输入:
%   DCM : 3x3 方向余弦矩阵，表示坐标系之间的姿态变换。
%
% 输出:
%   q   : 4x1 四元数，格式为 [q0; q1; q2; q3]，其中 q0 为标量项。
%
% 说明:
%   1. 本函数采用常见的“按迹分段”算法，从旋转矩阵中恢复四元数。
%   2. 当矩阵迹较大时，优先通过迹直接求解，可获得较好的数值稳定性。
%   3. 当迹较小时，转而根据对角线中最大的元素分支求解，避免分母过小。
%   4. 最后统一对结果做归一化，并强制 q0 >= 0，减少四元数正负号等价造成的跳变。

traceVal = trace(DCM);

if traceVal > 0
    s = 2.0 * sqrt(traceVal + 1.0);
    q0 = 0.25 * s;
    q1 = (DCM(3, 2) - DCM(2, 3)) / s;
    q2 = (DCM(1, 3) - DCM(3, 1)) / s;
    q3 = (DCM(2, 1) - DCM(1, 2)) / s;
elseif DCM(1, 1) > DCM(2, 2) && DCM(1, 1) > DCM(3, 3)
    s = 2.0 * sqrt(1.0 + DCM(1, 1) - DCM(2, 2) - DCM(3, 3));
    q0 = (DCM(3, 2) - DCM(2, 3)) / s;
    q1 = 0.25 * s;
    q2 = (DCM(1, 2) + DCM(2, 1)) / s;
    q3 = (DCM(1, 3) + DCM(3, 1)) / s;
elseif DCM(2, 2) > DCM(3, 3)
    s = 2.0 * sqrt(1.0 + DCM(2, 2) - DCM(1, 1) - DCM(3, 3));
    q0 = (DCM(1, 3) - DCM(3, 1)) / s;
    q1 = (DCM(1, 2) + DCM(2, 1)) / s;
    q2 = 0.25 * s;
    q3 = (DCM(2, 3) + DCM(3, 2)) / s;
else
    s = 2.0 * sqrt(1.0 + DCM(3, 3) - DCM(1, 1) - DCM(2, 2));
    q0 = (DCM(2, 1) - DCM(1, 2)) / s;
    q1 = (DCM(1, 3) + DCM(3, 1)) / s;
    q2 = (DCM(2, 3) + DCM(3, 2)) / s;
    q3 = 0.25 * s;
end

q = normalize_q([q0; q1; q2; q3]);
if q(1) < 0
    q = -q;
end
end