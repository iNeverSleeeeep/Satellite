function [actuatorOutput, actuatorState] = actuator_step(torqueCmd_b, actuatorConfig, actuatorState, dt, actuatorContext)
%ACTUATOR_STEP 执行器链路单步更新。
%
% 当前版本已完全切换到新的执行器子目录接口：
%   - reaction_wheel/reaction_wheel_step
%   - magnetorquer/magnetorquer_step
%   - thruster/thruster_step
% 不再依赖旧的兼容入口文件。

if nargin < 5 || isempty(actuatorContext)
    actuatorContext = struct();
end

if nargin < 3 || isempty(actuatorState)
    actuatorState = init_actuator_state(actuatorConfig);
end

if ~isfield(actuatorContext, 'magneticField_b') || isempty(actuatorContext.magneticField_b)
    actuatorContext.magneticField_b = zeros(3, 1);
end

selectedActuator = select_actuator_mode(torqueCmd_b, actuatorConfig, actuatorState, actuatorContext);
allocation = allocate_actuator_commands(torqueCmd_b, selectedActuator, actuatorConfig, actuatorContext);

[reactionWheelOutput, actuatorState.reactionWheel] = reaction_wheel_step( ...
    allocation.reactionWheel.commandedTorque_b, actuatorConfig.reactionWheel, actuatorState.reactionWheel, dt);
[magnetorquerOutput, actuatorState.magnetorquer] = magnetorquer_step( ...
    allocation.magnetorquer.commandedTorque_b, actuatorContext.magneticField_b, actuatorConfig.magnetorquer, actuatorState.magnetorquer, dt);
[thrusterOutput, actuatorState.thruster] = thruster_step( ...
    allocation.thruster.commandedTorque_b, actuatorConfig.thruster, actuatorState.thruster, dt);

actuatorState.selectedActuator = selectedActuator;

actuatorOutput.selectedActuator = selectedActuator;
actuatorOutput.requestedTorque_b = torqueCmd_b(:);
actuatorOutput.allocation = allocation;
actuatorOutput.reactionWheel = reactionWheelOutput;
actuatorOutput.magnetorquer = magnetorquerOutput;
actuatorOutput.thruster = thrusterOutput;
actuatorOutput.netTorque_b = reactionWheelOutput.actualTorque_b ...
    + magnetorquerOutput.actualTorque_b ...
    + thrusterOutput.actualTorque_b;
actuatorOutput.trackingError_b = torqueCmd_b(:) - actuatorOutput.netTorque_b;
end