function trackedValue = actuator_first_order_track(previousValue, commandValue, timeConstantS, dt)
%ACTUATOR_FIRST_ORDER_TRACK 执行器通用一阶跟踪环节。
%
% 用于描述执行器或驱动链的有限带宽：
%   x(k+1) = x(k) + alpha * (u - x(k))
%   alpha = dt / tau

if timeConstantS <= 0
    trackedValue = commandValue;
    return;
end

alpha = min(max(dt / max(timeConstantS, eps), 0.0), 1.0);
trackedValue = previousValue + alpha * (commandValue - previousValue);
end