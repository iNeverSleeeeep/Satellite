function gnc = gnc_overview()
%GNC_OVERVIEW GNC 配置总览。

gnc.guidanceMode = 'target-pointing';
gnc.estimator = 'attitude-ekf';
gnc.controller = 'pd';
gnc.sensorSuite = default_sensor_config();
gnc.observer = attitude_ekf_config();
end
