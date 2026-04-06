function [magnetorquerOutput, magnetorquerState] = magnetorquer_ideal(commandedTorque_b, magneticField_b, magnetorquerConfig, magnetorquerState, dt)
%MAGNETORQUER_IDEAL 磁力矩器理想/简化模型。

[projectedTorque_b, commandedDipoleAm2] = magnetorquer_project_torque(commandedTorque_b, magneticField_b, magnetorquerConfig);
actualDipoleAm2 = actuator_first_order_track(magnetorquerState.actualDipoleAm2(:), commandedDipoleAm2, magnetorquerConfig.timeConstantS, dt);
actualTorque_b = cross(actualDipoleAm2, magneticField_b(:));

magnetorquerOutput.model = 'ideal';
magnetorquerOutput.commandedTorque_b = commandedTorque_b(:);
magnetorquerOutput.projectedTorque_b = projectedTorque_b;
magnetorquerOutput.commandedDipoleAm2 = commandedDipoleAm2;
magnetorquerOutput.actualDipoleAm2 = actualDipoleAm2;
magnetorquerOutput.actualTorque_b = actualTorque_b;
magnetorquerOutput.coilCurrentCmdA = zeros(3, 1);
magnetorquerOutput.coilCurrentA = magnetorquerState.coilCurrentA(:);
magnetorquerOutput.driveVoltageCmdV = zeros(3, 1);
magnetorquerOutput.driveVoltageV = zeros(3, 1);
magnetorquerOutput.currentErrorA = zeros(3, 1);

magnetorquerState.actualDipoleAm2 = actualDipoleAm2;
magnetorquerState.actualTorque_b = actualTorque_b;
end