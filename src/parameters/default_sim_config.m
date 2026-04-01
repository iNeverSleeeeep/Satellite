function simConfig = default_sim_config()
%DEFAULT_SIM_CONFIG Baseline simulation configuration.

simConfig.name = 'baseline';
simConfig.startTime = 0.0;
simConfig.stopTime = 5400.0;
simConfig.sampleTime = 0.1;
simConfig.solver = 'ode45';
simConfig.enableDisturbance = true;
simConfig.enableSensorNoise = false;

simConfig.LG0 = 0.0; % Initial longitude (rad)
simConfig.omega_E = 7.292115e-5; % Earth's rotation rate (rad/s)
simConfig.X_i0 = [7000e3; 0; 0]; % Initial position in ECI (m)
simConfig.V_i0 = [0; 7546; 0]; % Initial velocity in ECI (m/s)
simConfig.q_b0 = [1; 0; 0; 0];
simConfig.omega_b0 = [0.0; 0; 0.0]; % Initial angular velocity in body frame (rad/s)

end
