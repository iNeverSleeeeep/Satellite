function [thrusterOutput, thrusterState] = simulate_thruster_actuator(commandedTorque_b, thrusterConfig, thrusterState, dt)
%SIMULATE_THRUSTER_ACTUATOR 推力器执行器模型。
%
% 当前状态：
%   - ideal   : 等效力矩模型，适合控制分配原型。
%   - physics : 增加阀门延迟和最小开机时间骨架，但仍使用等效力矩近似。
%
% 后续若要进一步逼近星上执行链，可把 physics 模式扩展为：
%   喷嘴开关 -> 推力 -> r x F -> 机体力矩。

commandedTorque_b = commandedTorque_b(:);
modelName = local_get_model_name(thrusterConfig);

limitedTorque_b = min(max(commandedTorque_b, -thrusterConfig.maxTorqueNm(:)), thrusterConfig.maxTorqueNm(:));
quantizedTorque_b = local_apply_min_pulse(limitedTorque_b, thrusterConfig.minPulseTorqueNm(:));

if strcmp(modelName, 'physics')
    [gateTorque_b, commandTimerS] = local_apply_valve_logic(quantizedTorque_b, thrusterConfig, thrusterState.commandTimerS(:), dt);
    actualTorque_b = local_first_order_track(thrusterState.actualTorque_b(:), gateTorque_b, thrusterConfig.timeConstantS, dt);
    thrusterState.commandTimerS = commandTimerS;
else
    gateTorque_b = quantizedTorque_b;
    actualTorque_b = local_first_order_track(thrusterState.actualTorque_b(:), quantizedTorque_b, thrusterConfig.timeConstantS, dt);
    thrusterState.commandTimerS = thrusterState.commandTimerS(:);
end

thrusterOutput.model = modelName;
thrusterOutput.commandedTorque_b = commandedTorque_b;
thrusterOutput.limitedTorque_b = limitedTorque_b;
thrusterOutput.quantizedTorque_b = quantizedTorque_b;
thrusterOutput.gateTorque_b = gateTorque_b;
thrusterOutput.actualTorque_b = actualTorque_b;

thrusterState.actualTorque_b = actualTorque_b;
end

function [gateTorque_b, stateTimer] = local_apply_valve_logic(commandedTorque_b, thrusterConfig, stateTimer, dt)
%LOCAL_APPLY_VALVE_LOGIC 简化推力器阀门逻辑。
%
% 含义：
%   - 非零命令需要经历阀门延迟，避免出现不现实的瞬时点火；
%   - 为了保持原型简单，这里先不细分推力建立曲线，只做时间门控。
gateTorque_b = zeros(3, 1);
for idx = 1:3
    if abs(commandedTorque_b(idx)) > 0
        stateTimer(idx) = stateTimer(idx) + dt;
        if stateTimer(idx) >= max(thrusterConfig.physics.valveDelayS, 0)
            gateTorque_b(idx) = commandedTorque_b(idx);
        end
    else
        stateTimer(idx) = 0.0;
    end
end
end

function quantizedTorque_b = local_apply_min_pulse(commandedTorque_b, minPulseTorqueNm)
quantizedTorque_b = commandedTorque_b;
activeMask = abs(commandedTorque_b) > 0;
belowMinMask = activeMask & abs(commandedTorque_b) < minPulseTorqueNm;
quantizedTorque_b(belowMinMask) = sign(commandedTorque_b(belowMinMask)) .* minPulseTorqueNm(belowMinMask);
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