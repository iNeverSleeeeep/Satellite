function actuatorState = init_actuator_state(actuatorConfig)
%INIT_ACTUATOR_STATE 初始化执行器内部状态。
%
% 说明：
%   为了同时支持 ideal / physics 两种模型，这里把理想模型状态和物理模型状态
%   一并初始化。未使用的字段不会影响后续仿真。

actuatorState.selectedActuator = actuatorConfig.defaultSelector;

% 飞轮状态。
actuatorState.reactionWheel.actualTorque_b = zeros(3, 1);
actuatorState.reactionWheel.momentumNms = actuatorConfig.reactionWheel.initialMomentumNms(:);
actuatorState.reactionWheel.motorCurrentA = zeros(3, 1);
actuatorState.reactionWheel.wheelSpeedRadS = actuatorConfig.reactionWheel.physics.initialWheelSpeedRadS(:);
actuatorState.reactionWheel.bodyTorque_b = zeros(3, 1);

% 磁力矩器状态。
actuatorState.magnetorquer.actualDipoleAm2 = zeros(3, 1);
actuatorState.magnetorquer.actualTorque_b = zeros(3, 1);
actuatorState.magnetorquer.coilCurrentA = zeros(3, 1);

% 推力器状态。
actuatorState.thruster.actualTorque_b = zeros(3, 1);
actuatorState.thruster.commandTimerS = zeros(3, 1);
end