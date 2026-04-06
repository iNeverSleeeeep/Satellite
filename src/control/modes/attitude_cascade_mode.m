function [torqueCmd_b, controlMeta] = attitude_cascade_mode(controllerInput, controllerConfig, modeName)
%ATTITUDE_CASCADE_MODE 串级姿态控制模式。
%
% 结构：
%   姿态外环 -> 目标角速度
%   角速度内环 -> 目标力矩

[omegaCmd_b, outerMeta] = attitude_outer_loop(controllerInput, controllerConfig.attitudeOuter, modeName);
[torqueCmd_b, rateMeta] = rate_inner_loop(controllerInput, controllerConfig.rateInner, omegaCmd_b, modeName);

controlMeta.outerLoop = outerMeta;
controlMeta.rateLoop = rateMeta;
controlMeta.omegaCmd_b = omegaCmd_b;
end