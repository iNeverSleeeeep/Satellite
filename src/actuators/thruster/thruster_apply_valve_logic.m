function [gateTorque_b, stateTimer] = thruster_apply_valve_logic(commandedTorque_b, thrusterConfig, stateTimer, dt)
%THRUSTER_APPLY_VALVE_LOGIC 简化推力器阀门逻辑。
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