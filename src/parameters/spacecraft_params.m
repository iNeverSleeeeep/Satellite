function spacecraft = spacecraft_params()
%SPACECRAFT_PARAMS Baseline spacecraft physical properties.

spacecraft.massKg = 12.0;
spacecraft.inertiaKgM2 = diag([0.15, 0.12, 0.10]);
spacecraft.centerOfMassM = [0.0; 0.0; 0.0];
spacecraft.maxTorqueNm = [0.02; 0.02; 0.02];
end
