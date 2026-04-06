function controllerState = init_controller_state(controllerConfig)
%INIT_CONTROLLER_STATE 初始化控制器状态。
%
% 当前控制器状态较轻量，主要保存：
%   - 当前模式
%   - B-dot 控制器上一拍磁场，用于计算磁场变化率

controllerState.activeMode = controllerConfig.defaultMode;
controllerState.detumble.previousMagneticField_b = zeros(3, 1);
end