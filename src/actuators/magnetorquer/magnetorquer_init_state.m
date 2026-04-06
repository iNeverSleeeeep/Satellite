function magnetorquerState = magnetorquer_init_state()
%MAGNETORQUER_INIT_STATE 初始化磁力矩器状态。
magnetorquerState.actualDipoleAm2 = zeros(3, 1);
magnetorquerState.actualTorque_b = zeros(3, 1);
magnetorquerState.coilCurrentA = zeros(3, 1);
end