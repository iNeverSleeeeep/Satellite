function innerLoopOutput = magnetorquer_inner_current_loop(commandedDipoleAm2, magnetorquerConfig, previousCurrentA, dt)
%MAGNETORQUER_INNER_CURRENT_LOOP 磁力矩器线圈电流内环与电气模型。

physicsCfg = magnetorquerConfig.physics;
currentCmdA = commandedDipoleAm2(:) ./ max(physicsCfg.dipolePerAmpAm2(:), eps);
currentCmdA = min(max(currentCmdA, -physicsCfg.maxCoilCurrentA(:)), physicsCfg.maxCoilCurrentA(:));

currentErrorA = currentCmdA - previousCurrentA(:);
driveVoltageCmdV = physicsCfg.currentControllerGain(:) .* currentErrorA;
driveVoltageV = min(max(driveVoltageCmdV, -physicsCfg.driveVoltageV(:)), physicsCfg.driveVoltageV(:));

coilInductance = max(physicsCfg.coilInductanceH(:), eps);
coilResistance = physicsCfg.coilResistanceOhm(:);
currentDotA = (driveVoltageV - coilResistance .* previousCurrentA(:)) ./ coilInductance;
coilCurrentA = previousCurrentA(:) + currentDotA * dt;
coilCurrentA = min(max(coilCurrentA, -physicsCfg.maxCoilCurrentA(:)), physicsCfg.maxCoilCurrentA(:));

innerLoopOutput.coilCurrentCmdA = currentCmdA;
innerLoopOutput.coilCurrentA = coilCurrentA;
innerLoopOutput.driveVoltageCmdV = driveVoltageCmdV;
innerLoopOutput.driveVoltageV = driveVoltageV;
innerLoopOutput.currentErrorA = currentErrorA;
innerLoopOutput.currentDotA = currentDotA;
end