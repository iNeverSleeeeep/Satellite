function [torqueCmd_b, detumbleState, controlMeta] = detumble_mode(controllerInput, detumbleConfig, detumbleState)
%DETUMBLE_MODE 基于 B-dot 的去旋控制模式。

magneticField_b = controllerInput.magneticField_b(:);
dt = max(controllerInput.dt, eps);

bDot_b = (magneticField_b - detumbleState.previousMagneticField_b(:)) / dt;
dipoleCmd = -detumbleConfig.kBdot .* bDot_b;
torqueCmd_b = cross(dipoleCmd, magneticField_b);
torqueCmd_b = min(max(torqueCmd_b, -detumbleConfig.maxTorqueNm(:)), detumbleConfig.maxTorqueNm(:));

detumbleState.previousMagneticField_b = magneticField_b;
controlMeta.bDot_b = bDot_b;
controlMeta.dipoleCmd = dipoleCmd;
end