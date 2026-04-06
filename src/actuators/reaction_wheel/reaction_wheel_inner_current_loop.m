function [innerLoopOutput, reactionWheelState] = reaction_wheel_inner_current_loop(motorCurrentCmdA, reactionWheelConfig, reactionWheelState, dt)
%REACTION_WHEEL_INNER_CURRENT_LOOP 飞轮电流内环。
%
% 结构：
%   1. 目标力矩先被外层转换成目标电流；
%   2. 这里使用 PI 控制器生成驱动电压；
%   3. 再用电机 R-L 电气模型计算实际电流。
%
% 这样可以清楚区分：
%   - 外层要什么
%   - 内层跟得上多少
%   - 是电压饱和导致误差，还是参数/动力学导致误差

phys = reactionWheelConfig.physics;
motorCurrentCmdA = min(max(motorCurrentCmdA(:), -phys.maxMotorCurrentA(:)), phys.maxMotorCurrentA(:));

currentErrorA = motorCurrentCmdA - reactionWheelState.motorCurrentA(:);
currentErrorIntegral = reactionWheelState.currentErrorIntegral(:) + currentErrorA * dt;
driveVoltageCmdV = phys.currentControllerKp(:) .* currentErrorA + phys.currentControllerKi(:) .* currentErrorIntegral;
driveVoltageV = min(max(driveVoltageCmdV, -phys.driveVoltageV(:)), phys.driveVoltageV(:));

voltageSaturated = abs(driveVoltageCmdV) > phys.driveVoltageV(:);
currentErrorIntegral(voltageSaturated) = reactionWheelState.currentErrorIntegral(voltageSaturated);

motorInductance = max(phys.motorInductanceH(:), eps);
currentDotA = (driveVoltageV - phys.motorResistanceOhm(:) .* reactionWheelState.motorCurrentA(:)) ./ motorInductance;
motorCurrentA = reactionWheelState.motorCurrentA(:) + currentDotA * dt;
motorCurrentA = min(max(motorCurrentA, -phys.maxMotorCurrentA(:)), phys.maxMotorCurrentA(:));

innerLoopOutput.motorCurrentCmdA = motorCurrentCmdA;
innerLoopOutput.motorCurrentA = motorCurrentA;
innerLoopOutput.currentErrorA = currentErrorA;
innerLoopOutput.currentErrorIntegral = currentErrorIntegral;
innerLoopOutput.driveVoltageCmdV = driveVoltageCmdV;
innerLoopOutput.driveVoltageV = driveVoltageV;
innerLoopOutput.currentDotA = currentDotA;

reactionWheelState.motorCurrentA = motorCurrentA;
reactionWheelState.currentErrorIntegral = currentErrorIntegral;
reactionWheelState.driveVoltageV = driveVoltageV;
end