function [magnetorquerOutput, magnetorquerState] = magnetorquer_physics(commandedTorque_b, magneticField_b, magnetorquerConfig, magnetorquerState, dt)
%MAGNETORQUER_PHYSICS 磁力矩器物理模型。

[projectedTorque_b, commandedDipoleAm2] = magnetorquer_project_torque(commandedTorque_b, magneticField_b, magnetorquerConfig);
innerLoopOutput = magnetorquer_inner_current_loop(commandedDipoleAm2, magnetorquerConfig, magnetorquerState.coilCurrentA(:), dt);
actualDipoleAm2 = magnetorquerConfig.physics.dipolePerAmpAm2(:) .* innerLoopOutput.coilCurrentA;
actualTorque_b = cross(actualDipoleAm2, magneticField_b(:));

magnetorquerOutput.model = 'physics';
magnetorquerOutput.commandedTorque_b = commandedTorque_b(:);
magnetorquerOutput.projectedTorque_b = projectedTorque_b;
magnetorquerOutput.commandedDipoleAm2 = commandedDipoleAm2;
magnetorquerOutput.actualDipoleAm2 = actualDipoleAm2;
magnetorquerOutput.actualTorque_b = actualTorque_b;
magnetorquerOutput.coilCurrentCmdA = innerLoopOutput.coilCurrentCmdA;
magnetorquerOutput.coilCurrentA = innerLoopOutput.coilCurrentA;
magnetorquerOutput.driveVoltageCmdV = innerLoopOutput.driveVoltageCmdV;
magnetorquerOutput.driveVoltageV = innerLoopOutput.driveVoltageV;
magnetorquerOutput.currentErrorA = innerLoopOutput.currentErrorA;

magnetorquerState.actualDipoleAm2 = actualDipoleAm2;
magnetorquerState.actualTorque_b = actualTorque_b;
magnetorquerState.coilCurrentA = innerLoopOutput.coilCurrentA;
end