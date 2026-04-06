function quantizedTorque_b = thruster_apply_min_pulse(commandedTorque_b, minPulseTorqueNm)
%THRUSTER_APPLY_MIN_PULSE 最小脉冲量化。
quantizedTorque_b = commandedTorque_b(:);
activeMask = abs(commandedTorque_b(:)) > 0;
belowMinMask = activeMask & abs(commandedTorque_b(:)) < minPulseTorqueNm(:);
quantizedTorque_b(belowMinMask) = sign(commandedTorque_b(belowMinMask)) .* minPulseTorqueNm(belowMinMask);
end