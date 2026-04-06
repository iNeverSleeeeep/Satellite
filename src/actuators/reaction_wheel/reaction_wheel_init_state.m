function reactionWheelState = reaction_wheel_init_state(reactionWheelConfig)
%REACTION_WHEEL_INIT_STATE 初始化飞轮执行器状态。
%
% 这里把飞轮相关状态独立放到子目录内管理，便于后续继续扩展：
%   - 内环控制状态
%   - 电气状态
%   - 机械状态
%   - 遥测/估计状态

reactionWheelState.actualTorque_b = zeros(3, 1);
reactionWheelState.momentumNms = reactionWheelConfig.initialMomentumNms(:);
reactionWheelState.motorCurrentA = zeros(3, 1);
reactionWheelState.currentErrorIntegral = zeros(3, 1);
reactionWheelState.driveVoltageV = zeros(3, 1);
reactionWheelState.wheelSpeedRadS = reactionWheelConfig.physics.initialWheelSpeedRadS(:);
reactionWheelState.bodyTorque_b = zeros(3, 1);
end