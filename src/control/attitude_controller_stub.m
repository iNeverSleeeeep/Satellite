function torqueCmd = attitude_controller_stub(attitudeError, rateError, gains)
%ATTITUDE_CONTROLLER_STUB Simple PD controller placeholder.

torqueCmd = -gains.kp .* attitudeError - gains.kd .* rateError;
end
