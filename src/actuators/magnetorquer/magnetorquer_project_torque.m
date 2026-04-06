function [projectedTorque_b, commandedDipoleAm2] = magnetorquer_project_torque(commandedTorque_b, magneticField_b, magnetorquerConfig)
%MAGNETORQUER_PROJECT_TORQUE 把目标力矩投影到磁力矩器可实现平面。

projectedTorque_b = zeros(3, 1);
commandedDipoleAm2 = zeros(3, 1);
magneticField_b = magneticField_b(:);
magNorm = norm(magneticField_b);

if magNorm <= magnetorquerConfig.minMagneticFieldNormT
    return;
end

bHat = magneticField_b / magNorm;
projectedTorque_b = commandedTorque_b(:) - bHat * dot(bHat, commandedTorque_b(:));
commandedDipoleAm2 = cross(magneticField_b, projectedTorque_b) / (magNorm^2);
commandedDipoleAm2 = min(max(commandedDipoleAm2, -magnetorquerConfig.maxDipoleAm2(:)), magnetorquerConfig.maxDipoleAm2(:));
end