function thrusterState = thruster_init_state()
%THRUSTER_INIT_STATE 初始化推力器状态。
thrusterState.actualTorque_b = zeros(3, 1);
thrusterState.commandTimerS = zeros(3, 1);
end