function [omegaCmd_b, controlMeta] = attitude_outer_loop(controllerInput, outerLoopConfig, modeName)
%ATTITUDE_OUTER_LOOP 姿态外环。
%
% 输入：当前姿态 q_bi、目标姿态 q_ref_bi。
% 输出：目标角速度 omegaCmd_b。

q_bi = controllerInput.q_bi(:);
q_ref_bi = controllerInput.q_ref_bi(:);

qErr = local_quat_multiply(local_quat_conjugate(q_ref_bi), q_bi);
qErr = qErr ./ max(norm(qErr), eps);
if qErr(1) < 0
    qErr = -qErr;
end

attitudeError_b = 2.0 * qErr(2:4);

switch char(lower(string(modeName)))
    case 'fine'
        kq = outerLoopConfig.kqFine(:);
    otherwise
        kq = outerLoopConfig.kqCoarse(:);
end

omegaCmd_b = -kq .* attitudeError_b;
omegaCmd_b = min(max(omegaCmd_b, -outerLoopConfig.maxOmegaCmdRadS(:)), outerLoopConfig.maxOmegaCmdRadS(:));

controlMeta.qError = qErr;
controlMeta.attitudeError_b = attitudeError_b;
controlMeta.kq = kq;
end

function qConj = local_quat_conjugate(q)
qConj = [q(1); -q(2:4)];
end

function qOut = local_quat_multiply(q1, q2)
qOut = [ ...
    q1(1) * q2(1) - dot(q1(2:4), q2(2:4));
    q1(1) * q2(2:4) + q2(1) * q1(2:4) + cross(q1(2:4), q2(2:4))];
end