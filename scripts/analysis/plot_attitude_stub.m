function plot_attitude_stub(time, angleDeg)
%PLOT_ATTITUDE_STUB Minimal plotting helper for attitude analysis.

figure;
plot(time, angleDeg, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Attitude Response');
end
