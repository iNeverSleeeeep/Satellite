function gnc = gnc_overview()
%GNC_OVERVIEW GNC configuration overview.
gnc.guidanceMode = 'target-pointing';
gnc.estimator = 'attitude-ekf';
gnc.controller = default_controller_config();
gnc.sensorSuite = default_sensor_config();
gnc.observer = attitude_ekf_config();
gnc.actuators = default_actuator_config();
end