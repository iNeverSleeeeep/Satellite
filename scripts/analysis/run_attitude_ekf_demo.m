function result = run_attitude_ekf_demo()
%RUN_ATTITUDE_EKF_DEMO Minimal demo for sun-sensor and magnetometer EKF attitude estimation.

setup_paths();
rng(0);

env = environment_config();
simConfig = default_sim_config();
gnc = gnc_overview();

t = 60.0;
dt = simConfig.sampleTime;
X_i = simConfig.X_i0;
qTrue_bi = normalize_q([0.88; 0.16; -0.09; 0.43]);
omega_b = [0.0; 0.0; 0.0];
state = [];

for k = 1:5
    [state, measurements] = estimate_attitude_from_sensors( ...
        t, X_i, qTrue_bi, omega_b, dt, env, gnc.sensorSuite, gnc.observer, state, false);
end

DCMTrue = q2dcm(qTrue_bi);
DCMEst = q2dcm(state.q_bi);
errorRad = acos(max(-1.0, min(1.0, 0.5 * (trace(DCMEst * DCMTrue') - 1.0))));

result.qTrue_bi = qTrue_bi;
result.qEst_bi = state.q_bi;
result.attitudeErrorDeg = rad2deg(errorRad);
result.measurements = measurements;

fprintf('Estimated quaternion: [% .6f % .6f % .6f % .6f]\n', state.q_bi);
fprintf('Attitude error: %.6f deg\n', result.attitudeErrorDeg);
end
