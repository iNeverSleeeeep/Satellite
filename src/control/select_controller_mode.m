function modeName = select_controller_mode(controllerInput, controllerConfig)
%SELECT_CONTROLLER_MODE 根据当前工况选择控制器模式。

if isfield(controllerInput, 'preferredMode') && ~isempty(controllerInput.preferredMode)
    modeName = char(lower(string(controllerInput.preferredMode)));
    return;
end

if ~strcmpi(controllerConfig.activeMode, 'auto')
    modeName = char(lower(string(controllerConfig.activeMode)));
    return;
end

omegaNorm = norm(controllerInput.omega_b(:));
attitudeErrorRad = local_calc_attitude_error(controllerInput.q_bi(:), controllerInput.q_ref_bi(:));
magNorm = norm(local_get_vector(controllerInput, 'magneticField_b'));

wheelMomentum = local_get_vector(controllerInput, 'wheelMomentumNms');
wheelMomentumMax = local_get_vector(controllerInput, 'wheelMomentumMaxNms');
if any(wheelMomentumMax > 0)
    momentumRatio = max(abs(wheelMomentum) ./ max(abs(wheelMomentumMax), eps));
else
    momentumRatio = 0.0;
end

if controllerConfig.detumble.enabled && omegaNorm >= controllerConfig.switch.detumbleRateThresholdRadS
    modeName = 'detumble';
    return;
end

if controllerConfig.momentumDump.enabled ...
        && momentumRatio >= controllerConfig.switch.momentumDumpRatio ...
        && magNorm > controllerConfig.switch.minMagneticFieldNormT
    modeName = 'momentum-dump';
    return;
end

if attitudeErrorRad >= controllerConfig.switch.coarseAttitudeErrorRad
    modeName = 'coarse-point';
elseif attitudeErrorRad <= controllerConfig.switch.fineAttitudeErrorRad
    modeName = 'fine-point';
else
    modeName = 'coarse-point';
end
end

function value = local_get_vector(dataStruct, fieldName)
value = zeros(3, 1);
if isfield(dataStruct, fieldName) && ~isempty(dataStruct.(fieldName))
    value = dataStruct.(fieldName)(:);
end
end

function attitudeErrorRad = local_calc_attitude_error(q_bi, q_ref_bi)
qErr = local_quat_multiply(local_quat_conjugate(q_ref_bi(:)), q_bi(:));
qErr = qErr ./ max(norm(qErr), eps);
attitudeErrorRad = 2 * atan2(norm(qErr(2:4)), abs(qErr(1)));
end

function qConj = local_quat_conjugate(q)
qConj = [q(1); -q(2:4)];
end

function qOut = local_quat_multiply(q1, q2)
qOut = [ ...
    q1(1) * q2(1) - dot(q1(2:4), q2(2:4));
    q1(1) * q2(2:4) + q2(1) * q1(2:4) + cross(q1(2:4), q2(2:4))];
end