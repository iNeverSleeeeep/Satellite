function trackedValue = thruster_first_order_track(previousValue, commandValue, timeConstantS, dt)
%THRUSTER_FIRST_ORDER_TRACK 一阶跟踪环节。
if timeConstantS <= 0
    trackedValue = commandValue;
    return;
end

alpha = min(max(dt / max(timeConstantS, eps), 0.0), 1.0);
trackedValue = previousValue + alpha * (commandValue - previousValue);
end