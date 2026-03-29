function simConfig = default_sim_config()
%DEFAULT_SIM_CONFIG Baseline simulation configuration.

simConfig.name = 'baseline';
simConfig.startTime = 0.0;
simConfig.stopTime = 5400.0;
simConfig.sampleTime = 0.1;
simConfig.solver = 'ode45';
simConfig.enableDisturbance = true;
simConfig.enableSensorNoise = false;
end
