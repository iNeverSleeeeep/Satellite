function [torqueCmd_b, controlMeta] = rate_inner_loop(controllerInput, rateLoopConfig, omegaCmd_b, modeName)
%RATE_INNER_LOOP 角速度内环。
%
% 输入：目标角速度 omegaCmd_b 和当前角速度 omega_b。
% 输出：目标控制力矩 torqueCmd_b。

omega_b = controllerInput.omega_b(:);
rateError_b = omega_b - omegaCmd_b(:);

switch char(lower(string(modeName)))
    case 'fine'
        kw = rateLoopConfig.kwFine(:);
    otherwise
        kw = rateLoopConfig.kwCoarse(:);
end

torqueCmd_b = -kw .* rateError_b;
torqueCmd_b = min(max(torqueCmd_b, -rateLoopConfig.maxTorqueNm(:)), rateLoopConfig.maxTorqueNm(:));

controlMeta.omegaCmd_b = omegaCmd_b(:);
controlMeta.rateError_b = rateError_b;
controlMeta.kw = kw;
end