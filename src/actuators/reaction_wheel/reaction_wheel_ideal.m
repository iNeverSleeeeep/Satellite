function [reactionWheelOutput, reactionWheelState] = reaction_wheel_ideal(commandedTorque_b, reactionWheelConfig, reactionWheelState, dt)
%REACTION_WHEEL_IDEAL 飞轮理想/简化模型。

limitedTorque_b = min(max(commandedTorque_b(:), -reactionWheelConfig.maxTorqueNm(:)), reactionWheelConfig.maxTorqueNm(:));

momentumNms = reactionWheelState.momentumNms(:);
maxMomentum = reactionWheelConfig.maxMomentumNms(:);
blockedAxes = abs(momentumNms) >= maxMomentum & sign(limitedTorque_b) == sign(momentumNms);
limitedTorque_b(blockedAxes) = 0.0;

actualTorque_b = actuator_first_order_track(reactionWheelState.actualTorque_b(:), limitedTorque_b, reactionWheelConfig.timeConstantS, dt);
momentumNms = min(max(momentumNms - actualTorque_b * dt, -maxMomentum), maxMomentum);

reactionWheelOutput.model = 'ideal';
reactionWheelOutput.commandedTorque_b = commandedTorque_b(:);
reactionWheelOutput.limitedTorque_b = limitedTorque_b;
reactionWheelOutput.actualTorque_b = actualTorque_b;
reactionWheelOutput.bodyTorque_b = actualTorque_b;
reactionWheelOutput.momentumNms = momentumNms;
reactionWheelOutput.motorCurrentCmdA = zeros(3, 1);
reactionWheelOutput.motorCurrentA = reactionWheelState.motorCurrentA(:);
reactionWheelOutput.currentErrorA = zeros(3, 1);
reactionWheelOutput.currentErrorIntegral = reactionWheelState.currentErrorIntegral(:);
reactionWheelOutput.driveVoltageCmdV = zeros(3, 1);
reactionWheelOutput.driveVoltageV = reactionWheelState.driveVoltageV(:);
reactionWheelOutput.wheelSpeedRadS = reactionWheelState.wheelSpeedRadS(:);

reactionWheelState.actualTorque_b = actualTorque_b;
reactionWheelState.momentumNms = momentumNms;
reactionWheelState.bodyTorque_b = actualTorque_b;
end