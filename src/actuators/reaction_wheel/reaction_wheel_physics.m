function [reactionWheelOutput, reactionWheelState] = reaction_wheel_physics(commandedTorque_b, reactionWheelConfig, reactionWheelState, dt)
%REACTION_WHEEL_PHYSICS 飞轮物理模型。

phys = reactionWheelConfig.physics;
maxTorque = reactionWheelConfig.maxTorqueNm(:);
commandedTorque_b = min(max(commandedTorque_b(:), -maxTorque), maxTorque);

motorCurrentCmdA = -commandedTorque_b ./ max(phys.motorTorqueConstantNmPerA(:), eps);
[innerLoopOutput, reactionWheelState] = reaction_wheel_inner_current_loop(motorCurrentCmdA, reactionWheelConfig, reactionWheelState, dt);

wheelSpeedRadS = reactionWheelState.wheelSpeedRadS(:);
wheelInertia = phys.wheelInertiaKgM2(:);
maxWheelSpeed = phys.maxWheelSpeedRadS(:);

motorTorqueNm = phys.motorTorqueConstantNmPerA(:) .* innerLoopOutput.motorCurrentA;
frictionTorqueNm = phys.viscousFrictionNmPerRadS(:) .* wheelSpeedRadS + phys.coulombFrictionNm(:) .* actuator_sign_with_zero(wheelSpeedRadS);
netWheelTorqueNm = motorTorqueNm - frictionTorqueNm;

blockedAxes = abs(wheelSpeedRadS) >= maxWheelSpeed & sign(netWheelTorqueNm) == sign(wheelSpeedRadS);
netWheelTorqueNm(blockedAxes) = 0.0;

wheelAccelRadS2 = netWheelTorqueNm ./ max(wheelInertia, eps);
wheelSpeedRadS = wheelSpeedRadS + wheelAccelRadS2 * dt;
wheelSpeedRadS = min(max(wheelSpeedRadS, -maxWheelSpeed), maxWheelSpeed);

bodyTorque_b = -netWheelTorqueNm;
actualTorque_b = min(max(bodyTorque_b, -maxTorque), maxTorque);

momentumNms = wheelInertia .* wheelSpeedRadS;
momentumNms = min(max(momentumNms, -reactionWheelConfig.maxMomentumNms(:)), reactionWheelConfig.maxMomentumNms(:));

reactionWheelOutput.model = 'physics';
reactionWheelOutput.commandedTorque_b = commandedTorque_b;
reactionWheelOutput.limitedTorque_b = commandedTorque_b;
reactionWheelOutput.actualTorque_b = actualTorque_b;
reactionWheelOutput.bodyTorque_b = bodyTorque_b;
reactionWheelOutput.momentumNms = momentumNms;
reactionWheelOutput.motorCurrentCmdA = innerLoopOutput.motorCurrentCmdA;
reactionWheelOutput.motorCurrentA = innerLoopOutput.motorCurrentA;
reactionWheelOutput.currentErrorA = innerLoopOutput.currentErrorA;
reactionWheelOutput.currentErrorIntegral = innerLoopOutput.currentErrorIntegral;
reactionWheelOutput.driveVoltageCmdV = innerLoopOutput.driveVoltageCmdV;
reactionWheelOutput.driveVoltageV = innerLoopOutput.driveVoltageV;
reactionWheelOutput.motorTorqueNm = motorTorqueNm;
reactionWheelOutput.frictionTorqueNm = frictionTorqueNm;
reactionWheelOutput.wheelSpeedRadS = wheelSpeedRadS;
reactionWheelOutput.wheelAccelRadS2 = wheelAccelRadS2;

reactionWheelState.actualTorque_b = actualTorque_b;
reactionWheelState.bodyTorque_b = bodyTorque_b;
reactionWheelState.momentumNms = momentumNms;
reactionWheelState.wheelSpeedRadS = wheelSpeedRadS;
end