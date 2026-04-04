function test_attitude_estimation()
%TEST_ATTITUDE_ESTIMATION 太阳敏感器 + 磁力计 + EKF 姿态估计冒烟测试。

setup_paths();
rng(0);

env = environment_config();
simConfig = default_sim_config();
gnc = gnc_overview();

t = 25.0;
dt = simConfig.sampleTime;
X_i = simConfig.X_i0;
qTrue_bi = normalize_q([0.91; 0.12; -0.08; 0.38]);
omega_b = [0.0; 0.0; 0.0];

measurements = measure_attitude_sensors(t, X_i, qTrue_bi, env, gnc.sensorSuite, false);
state = [];

for k = 1:3
    state = attitude_ekf_step(state, omega_b, dt, measurements, gnc.observer);
end

DCMTrue = q2dcm(qTrue_bi);
DCMEst = q2dcm(state.q_bi);
attitudeErrorRad = acos(max(-1.0, min(1.0, 0.5 * (trace(DCMEst * DCMTrue') - 1.0))));

assert(measurements.sun.valid, 'Sun sensor measurement must be valid.');
assert(measurements.mag.valid, 'Magnetometer measurement must be valid.');
assert(all(size(state.q_bi) == [4, 1]), 'EKF must output a quaternion.');
assert(isfinite(attitudeErrorRad), 'Attitude error must be finite.');
assert(attitudeErrorRad < deg2rad(0.5), 'EKF estimate should remain close to the true attitude.');
end
