function sunVector_i = calc_sun_vector_i(t, env)
%CALC_SUN_VECTOR_I 计算惯性系下的太阳方向向量。
%
% 模型切换:
%   env.sunModel = 'simple'
%   env.sunModel = 'engineering'
%   env.sunModel = 'high_fidelity'
%
% high_fidelity 模式说明:
%   1. 若 env.highFidelity.sunFcn 为函数句柄，则直接调用该函数。
%   2. 否则优先尝试使用 MATLAB/Aerospace Toolbox 的 planetEphemeris。
%   3. 若相关能力不可用，则直接报错，而不是退化为近似模型。

modelName = local_get_model_name(env, 'sunModel', 'simple');

switch lower(modelName)
    case 'simple'
        sun0_i = local_unit_vector(env.sunVector0_i);
        theta = env.sunAngularRateRadS * t;
        Rz = [cos(theta), -sin(theta), 0.0; ...
              sin(theta),  cos(theta), 0.0; ...
              0.0,         0.0,        1.0];
        sunVector_i = Rz * sun0_i;

    case 'engineering'
        julianDate = env.referenceEpochJulianDate + t / 86400.0;
        sunVector_i = local_calc_sun_vector_engineering(julianDate);

    case 'high_fidelity'
        sunVector_i = local_calc_sun_vector_high_fidelity(t, env);

    otherwise
        error('calc_sun_vector_i:UnsupportedModel', ...
            'Unsupported sun model: %s', modelName);
end

sunVector_i = local_unit_vector(sunVector_i);
end

function sunVector_i = local_calc_sun_vector_engineering(julianDate)
daysFromJ2000 = julianDate - 2451545.0;
meanLongitudeDeg = mod(280.460 + 0.9856474 * daysFromJ2000, 360.0);
meanAnomalyDeg = mod(357.528 + 0.9856003 * daysFromJ2000, 360.0);

meanAnomalyRad = deg2rad(meanAnomalyDeg);
eclipticLongitudeDeg = meanLongitudeDeg ...
    + 1.915 * sin(meanAnomalyRad) ...
    + 0.020 * sin(2.0 * meanAnomalyRad);
eclipticLongitudeRad = deg2rad(eclipticLongitudeDeg);

obliquityDeg = 23.439 - 0.0000004 * daysFromJ2000;
obliquityRad = deg2rad(obliquityDeg);

sunVector_i = [cos(eclipticLongitudeRad); ...
               cos(obliquityRad) * sin(eclipticLongitudeRad); ...
               sin(obliquityRad) * sin(eclipticLongitudeRad)];
end

function sunVector_i = local_calc_sun_vector_high_fidelity(t, env)
%LOCAL_CALC_SUN_VECTOR_HIGH_FIDELITY 高保真太阳参考方向。
if isfield(env, 'highFidelity') && isfield(env.highFidelity, 'sunFcn') && isa(env.highFidelity.sunFcn, 'function_handle')
    sunVector_i = env.highFidelity.sunFcn(t, env);
    return;
end

if exist('planetEphemeris', 'file') == 2
    julianDate = env.referenceEpochJulianDate + t / 86400.0;
    ephemerisModel = '421';
    if isfield(env, 'highFidelity') && isfield(env.highFidelity, 'ephemerisModel') && ~isempty(env.highFidelity.ephemerisModel)
        ephemerisModel = env.highFidelity.ephemerisModel;
    end

    try
        earthToSun = planetEphemeris(julianDate, 'Earth', 'Sun', ephemerisModel);
    catch
        earthToSun = planetEphemeris(julianDate, 'Earth', 'Sun');
    end

    sunVector_i = earthToSun(:);
    return;
end

error('calc_sun_vector_i:HighFidelityUnavailable', ...
    ['High-fidelity sun model requires either env.highFidelity.sunFcn ' ...
     'or MATLAB function planetEphemeris.']);
end

function modelName = local_get_model_name(env, fieldName, defaultValue)
if isfield(env, fieldName) && ~isempty(env.(fieldName))
    modelName = env.(fieldName);
else
    modelName = defaultValue;
end
end