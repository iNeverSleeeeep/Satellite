function innerLoopOutput = thruster_inner_pulse_controller(commandedTorque_b, thrusterConfig, thrusterState, dt)
%THRUSTER_INNER_PULSE_CONTROLLER 推力器执行内环。
%
% 推力器通常不是连续电流内环，而是“离散脉冲执行内环”：
%   1. 对小力矩做最小脉冲量化；
%   2. 经阀门逻辑决定本拍是否真正点火；
%   3. 输出门控后的等效力矩给后续物理/执行模型。

limitedTorque_b = min(max(commandedTorque_b(:), -thrusterConfig.maxTorqueNm(:)), thrusterConfig.maxTorqueNm(:));
quantizedTorque_b = thruster_apply_min_pulse(limitedTorque_b, thrusterConfig.minPulseTorqueNm(:));
[gateTorque_b, commandTimerS] = thruster_apply_valve_logic(quantizedTorque_b, thrusterConfig, thrusterState.commandTimerS(:), dt);

innerLoopOutput.limitedTorque_b = limitedTorque_b;
innerLoopOutput.quantizedTorque_b = quantizedTorque_b;
innerLoopOutput.gateTorque_b = gateTorque_b;
innerLoopOutput.commandTimerS = commandTimerS;
end