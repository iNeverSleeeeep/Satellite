function [reactionWheelOutput, reactionWheelState] = simulate_reaction_wheel_actuator(commandedTorque_b, reactionWheelConfig, reactionWheelState, dt)
%SIMULATE_REACTION_WHEEL_ACTUATOR 飞轮执行器模型。
%
% 双模式说明：
%   - ideal   : 输入力矩直接经过限幅、饱和和一阶响应得到输出。
%   - physics : 输入力矩先转换成“等效电机电流命令”，再通过
%               电流建立、摩擦、轮速和角动量约束计算真实输出。
%
% physics 模式更接近星上实际链路：
%   torque_cmd -> motor_current_cmd -> motor_torque -> wheel_speed -> body_torque

commandedTorque_b = commandedTorque_b(:);
modelName = local_get_model_name(reactionWheelConfig);

switch modelName
    case 'physics'
        [reactionWheelOutput, reactionWheelState] = local_simulate_physics(commandedTorque_b, reactionWheelConfig, reactionWheelState, dt);
    otherwise
        [reactionWheelOutput, reactionWheelState] = local_simulate_ideal(commandedTorque_b, reactionWheelConfig, reactionWheelState, dt);
end
end

function [reactionWheelOutput, reactionWheelState] = local_simulate_ideal(commandedTorque_b, reactionWheelConfig, reactionWheelState, dt)
limitedTorque_b = min(max(commandedTorque_b, -reactionWheelConfig.maxTorqueNm(:)), reactionWheelConfig.maxTorqueNm(:));

momentumNms = reactionWheelState.momentumNms(:);
maxMomentum = reactionWheelConfig.maxMomentumNms(:);
blockedAxes = abs(momentumNms) >= maxMomentum & sign(limitedTorque_b) == sign(momentumNms);
limitedTorque_b(blockedAxes) = 0.0;

actualTorque_b = local_first_order_track(reactionWheelState.actualTorque_b(:), limitedTorque_b, reactionWheelConfig.timeConstantS, dt);
momentumNms = min(max(momentumNms - actualTorque_b * dt, -maxMomentum), maxMomentum);

reactionWheelOutput.model = 'ideal';
reactionWheelOutput.commandedTorque_b = commandedTorque_b;
reactionWheelOutput.limitedTorque_b = limitedTorque_b;
reactionWheelOutput.actualTorque_b = actualTorque_b;
reactionWheelOutput.bodyTorque_b = actualTorque_b;
reactionWheelOutput.momentumNms = momentumNms;
reactionWheelOutput.motorCurrentCmdA = zeros(3, 1);
reactionWheelOutput.motorCurrentA = reactionWheelState.motorCurrentA(:);
reactionWheelOutput.wheelSpeedRadS = reactionWheelState.wheelSpeedRadS(:);

reactionWheelState.actualTorque_b = actualTorque_b;
reactionWheelState.momentumNms = momentumNms;
reactionWheelState.bodyTorque_b = actualTorque_b;
end

function [reactionWheelOutput, reactionWheelState] = local_simulate_physics(commandedTorque_b, reactionWheelConfig, reactionWheelState, dt)
phys = reactionWheelConfig.physics;
maxTorque = reactionWheelConfig.maxTorqueNm(:);
commandedTorque_b = min(max(commandedTorque_b, -maxTorque), maxTorque);

% 1. 将目标机体力矩换算成等效电机电流命令。
% 对飞轮而言，机体力矩与飞轮电机力矩方向相反，因此这里取负号。
motorCurrentCmdA = -commandedTorque_b ./ max(phys.motorTorqueConstantNmPerA(:), eps);
motorCurrentCmdA = min(max(motorCurrentCmdA, -phys.maxMotorCurrentA(:)), phys.maxMotorCurrentA(:));

% 2. 电流环/驱动链的一阶响应。
motorCurrentA = local_first_order_track(reactionWheelState.motorCurrentA(:), motorCurrentCmdA, phys.driveTimeConstantS, dt);

wheelSpeedRadS = reactionWheelState.wheelSpeedRadS(:);
wheelInertia = phys.wheelInertiaKgM2(:);
maxWheelSpeed = phys.maxWheelSpeedRadS(:);

% 3. 根据电流生成电机力矩，再扣除摩擦力矩。
motorTorqueNm = phys.motorTorqueConstantNmPerA(:) .* motorCurrentA;
frictionTorqueNm = phys.viscousFrictionNmPerRadS(:) .* wheelSpeedRadS + phys.coulombFrictionNm(:) .* sign_with_zero(wheelSpeedRadS);
netWheelTorqueNm = motorTorqueNm - frictionTorqueNm;

% 4. 若轮速已到极限且净力矩还想继续同向加速，则阻断该轴继续加速。
blockedAxes = abs(wheelSpeedRadS) >= maxWheelSpeed & sign(netWheelTorqueNm) == sign(wheelSpeedRadS);
netWheelTorqueNm(blockedAxes) = 0.0;

% 5. 更新轮速，并再次做限幅。
wheelAccelRadS2 = netWheelTorqueNm ./ max(wheelInertia, eps);
wheelSpeedRadS = wheelSpeedRadS + wheelAccelRadS2 * dt;
wheelSpeedRadS = min(max(wheelSpeedRadS, -maxWheelSpeed), maxWheelSpeed);

% 6. 机体实际得到的控制力矩等于飞轮净力矩的反作用。
bodyTorque_b = -netWheelTorqueNm;
actualTorque_b = min(max(bodyTorque_b, -maxTorque), maxTorque);

% 7. 用轮速重新计算飞轮角动量，保证与物理状态一致。
momentumNms = wheelInertia .* wheelSpeedRadS;
momentumNms = min(max(momentumNms, -reactionWheelConfig.maxMomentumNms(:)), reactionWheelConfig.maxMomentumNms(:));

reactionWheelOutput.model = 'physics';
reactionWheelOutput.commandedTorque_b = commandedTorque_b;
reactionWheelOutput.limitedTorque_b = commandedTorque_b;
reactionWheelOutput.actualTorque_b = actualTorque_b;
reactionWheelOutput.bodyTorque_b = bodyTorque_b;
reactionWheelOutput.momentumNms = momentumNms;
reactionWheelOutput.motorCurrentCmdA = motorCurrentCmdA;
reactionWheelOutput.motorCurrentA = motorCurrentA;
reactionWheelOutput.motorTorqueNm = motorTorqueNm;
reactionWheelOutput.frictionTorqueNm = frictionTorqueNm;
reactionWheelOutput.wheelSpeedRadS = wheelSpeedRadS;
reactionWheelOutput.wheelAccelRadS2 = wheelAccelRadS2;

reactionWheelState.actualTorque_b = actualTorque_b;
reactionWheelState.bodyTorque_b = bodyTorque_b;
reactionWheelState.momentumNms = momentumNms;
reactionWheelState.motorCurrentA = motorCurrentA;
reactionWheelState.wheelSpeedRadS = wheelSpeedRadS;
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

function value = sign_with_zero(vectorIn)
value = sign(vectorIn);
value(abs(vectorIn) < eps) = 0.0;
end