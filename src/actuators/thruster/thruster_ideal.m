function [thrusterOutput, thrusterState] = thruster_ideal(commandedTorque_b, thrusterConfig, thrusterState, dt)
%THRUSTER_IDEAL 推力器理想/简化模型。

innerLoopOutput = thruster_inner_pulse_controller(commandedTorque_b, thrusterConfig, thrusterState, dt);
actualTorque_b = actuator_first_order_track(thrusterState.actualTorque_b(:), innerLoopOutput.quantizedTorque_b, thrusterConfig.timeConstantS, dt);

thrusterOutput.model = 'ideal';
thrusterOutput.commandedTorque_b = commandedTorque_b(:);
thrusterOutput.limitedTorque_b = innerLoopOutput.limitedTorque_b;
thrusterOutput.quantizedTorque_b = innerLoopOutput.quantizedTorque_b;
thrusterOutput.gateTorque_b = innerLoopOutput.quantizedTorque_b;
thrusterOutput.actualTorque_b = actualTorque_b;

thrusterState.actualTorque_b = actualTorque_b;
end