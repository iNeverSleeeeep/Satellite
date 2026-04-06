function [thrusterOutput, thrusterState] = thruster_physics(commandedTorque_b, thrusterConfig, thrusterState, dt)
%THRUSTER_PHYSICS 推力器物理骨架模型。
%
% 这里把“脉冲/阀门执行逻辑”明确作为推力器内环，
% 再由执行动态输出实际力矩。

innerLoopOutput = thruster_inner_pulse_controller(commandedTorque_b, thrusterConfig, thrusterState, dt);
actualTorque_b = actuator_first_order_track(thrusterState.actualTorque_b(:), innerLoopOutput.gateTorque_b, thrusterConfig.timeConstantS, dt);

thrusterOutput.model = 'physics';
thrusterOutput.commandedTorque_b = commandedTorque_b(:);
thrusterOutput.limitedTorque_b = innerLoopOutput.limitedTorque_b;
thrusterOutput.quantizedTorque_b = innerLoopOutput.quantizedTorque_b;
thrusterOutput.gateTorque_b = innerLoopOutput.gateTorque_b;
thrusterOutput.actualTorque_b = actualTorque_b;

thrusterState.actualTorque_b = actualTorque_b;
thrusterState.commandTimerS = innerLoopOutput.commandTimerS;
end