function spacecraft = spacecraft_params()
%SPACECRAFT_PARAMS 航天器基础物理参数。

spacecraft.massKg = 12.0;
spacecraft.inertiaKgM2 = diag([0.15, 0.12, 0.10]);
spacecraft.centerOfMassM = [0.0; 0.0; 0.0];
spacecraft.maxTorqueNm = [0.02; 0.02; 0.02];
spacecraft.dragCoefficient = 2.2;
spacecraft.dragAreaM2 = 0.08;
spacecraft.solarPressureCoefficient = 1.3;
spacecraft.solarAreaM2 = 0.10;
spacecraft.centerOfPressureM = [0.02; 0.00; 0.01];
spacecraft.centerOfDragM = [-0.01; 0.01; 0.00];
spacecraft.residualDipoleAm2 = [0.02; -0.01; 0.015];
end
