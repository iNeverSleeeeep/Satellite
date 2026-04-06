function actuatorState = init_actuator_state(actuatorConfig)
%INIT_ACTUATOR_STATE 初始化执行器内部状态。
%
% 说明：
%   每类执行器的局部状态初始化已下沉到各自子目录，
%   这里仅负责聚合整个执行器子系统的总状态结构。

actuatorState.selectedActuator = actuatorConfig.defaultSelector;
actuatorState.reactionWheel = reaction_wheel_init_state(actuatorConfig.reactionWheel);
actuatorState.magnetorquer = magnetorquer_init_state();
actuatorState.thruster = thruster_init_state();
end