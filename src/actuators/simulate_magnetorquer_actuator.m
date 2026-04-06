function [magnetorquerOutput, magnetorquerState] = simulate_magnetorquer_actuator(commandedTorque_b, magneticField_b, magnetorquerConfig, magnetorquerState, dt)
%SIMULATE_MAGNETORQUER_ACTUATOR 磁力矩器执行器模型。
%
% 双模式说明：
%   - ideal   : 直接把目标力矩换算为磁矩，并用一阶环节逼近。
%   - physics : 先根据目标磁矩生成电流命令，再通过电流环、电压限幅、
%               线圈 R-L 动态计算真实电流，最后得到真实磁偶极矩和输出力矩。
%
% physics 模式的链路更接近星上实际执行过程：
%   torque_cmd -> dipole_cmd -> current_cmd -> drive_voltage -> coil_current -> dipole -> torque

commandedTorque_b = commandedTorque_b(:);
magneticField_b = magneticField_b(:);
magNorm = norm(magneticField_b);
modelName = local_get_model_name(magnetorquerConfig);

commandedDipoleAm2 = zeros(3, 1);
actualDipoleAm2 = zeros(3, 1);
actualTorque_b = zeros(3, 1);
projectedTorque_b = zeros(3, 1);
coilCurrentCmdA = zeros(3, 1);
coilCurrentA = magnetorquerState.coilCurrentA(:);
driveVoltageCmdV = zeros(3, 1);
driveVoltageV = zeros(3, 1);
currentErrorA = zeros(3, 1);

if magNorm > magnetorquerConfig.minMagneticFieldNormT
    % 先将目标力矩投影到磁力矩器可实现的平面内。
    bHat = magneticField_b / magNorm;
    projectedTorque_b = commandedTorque_b - bHat * dot(bHat, commandedTorque_b);
    commandedDipoleAm2 = cross(magneticField_b, projectedTorque_b) / (magNorm^2);
    commandedDipoleAm2 = min(max(commandedDipoleAm2, -magnetorquerConfig.maxDipoleAm2(:)), magnetorquerConfig.maxDipoleAm2(:));

    if strcmp(modelName, 'physics')
        [coilCurrentCmdA, coilCurrentA, driveVoltageCmdV, driveVoltageV, currentErrorA] = local_simulate_coil_current( ...
            commandedDipoleAm2, magnetorquerConfig.physics, magnetorquerState.coilCurrentA(:), dt);
        actualDipoleAm2 = magnetorquerConfig.physics.dipolePerAmpAm2(:) .* coilCurrentA;
    else
        actualDipoleAm2 = local_first_order_track(magnetorquerState.actualDipoleAm2(:), commandedDipoleAm2, magnetorquerConfig.timeConstantS, dt);
    end

    actualTorque_b = cross(actualDipoleAm2, magneticField_b);
else
    if strcmp(modelName, 'physics')
        [coilCurrentCmdA, coilCurrentA, driveVoltageCmdV, driveVoltageV, currentErrorA] = local_simulate_coil_current( ...
            zeros(3, 1), magnetorquerConfig.physics, magnetorquerState.coilCurrentA(:), dt);
        actualDipoleAm2 = magnetorquerConfig.physics.dipolePerAmpAm2(:) .* coilCurrentA;
    else
        actualDipoleAm2 = local_first_order_track(magnetorquerState.actualDipoleAm2(:), zeros(3, 1), magnetorquerConfig.timeConstantS, dt);
    end
end

magnetorquerOutput.model = modelName;
magnetorquerOutput.commandedTorque_b = commandedTorque_b;
magnetorquerOutput.projectedTorque_b = projectedTorque_b;
magnetorquerOutput.commandedDipoleAm2 = commandedDipoleAm2;
magnetorquerOutput.actualDipoleAm2 = actualDipoleAm2;
magnetorquerOutput.actualTorque_b = actualTorque_b;
magnetorquerOutput.coilCurrentCmdA = coilCurrentCmdA;
magnetorquerOutput.coilCurrentA = coilCurrentA;
magnetorquerOutput.driveVoltageCmdV = driveVoltageCmdV;
magnetorquerOutput.driveVoltageV = driveVoltageV;
magnetorquerOutput.currentErrorA = currentErrorA;

magnetorquerState.actualDipoleAm2 = actualDipoleAm2;
magnetorquerState.actualTorque_b = actualTorque_b;
magnetorquerState.coilCurrentA = coilCurrentA;
end

function [currentCmdA, currentA, driveVoltageCmdV, driveVoltageV, currentErrorA] = local_simulate_coil_current(commandedDipoleAm2, physicsCfg, previousCurrentA, dt)
%LOCAL_SIMULATE_COIL_CURRENT 磁力矩器线圈电流物理模型。
%
% 建模思路：
%   1. 由目标磁偶极矩反推需要的目标线圈电流。
%   2. 用比例电流控制器生成驱动电压命令。
%   3. 驱动电压受电源能力限制。
%   4. 线圈满足 R-L 方程：
%          L * di/dt + R * i = V
%      因而：
%          di/dt = (V - R*i) / L
%   5. 再对电流做最大允许值限制。
currentCmdA = commandedDipoleAm2 ./ max(physicsCfg.dipolePerAmpAm2(:), eps);
currentCmdA = min(max(currentCmdA, -physicsCfg.maxCoilCurrentA(:)), physicsCfg.maxCoilCurrentA(:));

currentErrorA = currentCmdA - previousCurrentA(:);
driveVoltageCmdV = physicsCfg.currentControllerGain(:) .* currentErrorA;
driveVoltageV = min(max(driveVoltageCmdV, -physicsCfg.driveVoltageV(:)), physicsCfg.driveVoltageV(:));

coilInductance = max(physicsCfg.coilInductanceH(:), eps);
coilResistance = physicsCfg.coilResistanceOhm(:);
currentDotA = (driveVoltageV - coilResistance .* previousCurrentA(:)) ./ coilInductance;
currentA = previousCurrentA(:) + currentDotA * dt;
currentA = min(max(currentA, -physicsCfg.maxCoilCurrentA(:)), physicsCfg.maxCoilCurrentA(:));
end

function modelName = local_get_model_name(config)
modelName = 'ideal';
if isfield(config, 'model') && ~isempty(config.model)
    modelName = char(lower(string(config.model)));
end
end

function trackedValue = local_first_order_track(previousValue, commandValue, timeConstantS, dt)
if timeConstantS <= 0
    trackedValue = commandValue;
    return;
end

alpha = min(max(dt / max(timeConstantS, eps), 0.0), 1.0);
trackedValue = previousValue + alpha * (commandValue - previousValue);
end