function B_i = calc_magnetic_field_i(X_i, env, t)
%CALC_MAGNETIC_FIELD_I 计算惯性系下的地磁场向量。
%
% 模型切换:
%   env.magneticModel = 'simple'
%   env.magneticModel = 'engineering'
%   env.magneticModel = 'high_fidelity'
%
% high_fidelity 模式说明:
%   1. 若 env.highFidelity.magneticFcn 为函数句柄，则直接调用该函数。
%   2. 否则尝试使用 MATLAB/Aerospace Toolbox 的 IGRF/WMM 接口。
%   3. 若相关能力不可用，则直接报错，而不是退化为近似模型。

if nargin < 3
    t = 0.0;
end

modelName = local_get_model_name(env, 'magneticModel', 'simple');

switch lower(modelName)
    case 'simple'
        B_i = local_calc_magnetic_field_simple(X_i, env);

    case 'engineering'
        B_i = local_calc_magnetic_field_engineering(X_i, env, t);

    case 'high_fidelity'
        B_i = local_calc_magnetic_field_high_fidelity(X_i, env, t);

    otherwise
        error('calc_magnetic_field_i:UnsupportedModel', ...
            'Unsupported magnetic model: %s', modelName);
end
end

function B_i = local_calc_magnetic_field_simple(X_i, env)
mu0 = 4.0 * pi * 1e-7;
r = norm(X_i);

if r < eps
    B_i = zeros(3, 1);
    return;
end

r_hat_i = X_i / r;
m_hat_i = local_unit_vector(env.earthDipoleAxis_i);
m_i = env.earthMagneticDipoleAm2 * m_hat_i;
B_i = mu0 / (4.0 * pi * r^3) * (3.0 * r_hat_i * dot(m_i, r_hat_i) - m_i);
end

function B_i = local_calc_magnetic_field_engineering(X_i, env, t)
mu0 = 4.0 * pi * 1e-7;
r = norm(X_i);

if r < eps
    B_i = zeros(3, 1);
    return;
end

theta = env.earthRotationAtEpochRad + env.earthRotationRadS * t;
R_ie = local_rot_z(theta);
R_ei = R_ie';
X_e = R_ei * X_i;
r_hat_e = X_e / r;
m_hat_e = local_unit_vector(env.earthDipoleAxis_e);
m_e = env.earthMagneticDipoleAm2 * m_hat_e;
B_e = mu0 / (4.0 * pi * r^3) * (3.0 * r_hat_e * dot(m_e, r_hat_e) - m_e);
B_i = R_ie * B_e;
end

function B_i = local_calc_magnetic_field_high_fidelity(X_i, env, t)
%LOCAL_CALC_MAGNETIC_FIELD_HIGH_FIDELITY 高保真地磁参考向量。
if isfield(env, 'highFidelity') && isfield(env.highFidelity, 'magneticFcn') && isa(env.highFidelity.magneticFcn, 'function_handle')
    B_i = env.highFidelity.magneticFcn(X_i, t, env);
    return;
end

julianDate = env.referenceEpochJulianDate + t / 86400.0;
X_ecef = local_eci_to_ecef(X_i, julianDate);
[latDeg, lonDeg, altM] = local_ecef_to_geodetic_wgs84(X_ecef);
decimalYear = local_decimal_year_from_jd(julianDate);

if exist('igrfmagm', 'file') == 2
    B_ned_nT = local_call_igrfmagm(altM / 1000.0, latDeg, lonDeg, decimalYear);
elseif exist('wrldmagm', 'file') == 2
    B_ned_nT = local_call_wrldmagm(altM / 1000.0, latDeg, lonDeg, decimalYear);
else
    error('calc_magnetic_field_i:HighFidelityUnavailable', ...
        ['High-fidelity magnetic model requires either env.highFidelity.magneticFcn ' ...
         'or MATLAB function igrfmagm/wrldmagm.']);
end

B_ecef = local_ned_to_ecef(B_ned_nT(:) * 1e-9, latDeg, lonDeg);
B_i = local_ecef_to_eci(B_ecef, julianDate);
end

function B_ned_nT = local_call_igrfmagm(altKm, latDeg, lonDeg, decimalYear)
%LOCAL_CALL_IGRFMAGM 封装 igrfmagm 的可能调用形式。
try
    [B_ned_nT, ~, ~, ~] = igrfmagm(altKm, latDeg, lonDeg, decimalYear);
catch
    B_ned_nT = igrfmagm(altKm, latDeg, lonDeg, decimalYear);
end
B_ned_nT = B_ned_nT(:);
end

function B_ned_nT = local_call_wrldmagm(altKm, latDeg, lonDeg, decimalYear)
%LOCAL_CALL_WRLDMAGM 封装 wrldmagm 的可能调用形式。
try
    [B_ned_nT, ~, ~, ~] = wrldmagm(altKm, latDeg, lonDeg, decimalYear);
catch
    try
        [B_ned_nT, ~, ~, ~] = wrldmagm(altKm, latDeg, lonDeg, decimalYear, '2020');
    catch
        B_ned_nT = wrldmagm(altKm, latDeg, lonDeg, decimalYear);
    end
end
B_ned_nT = B_ned_nT(:);
end

function X_ecef = local_eci_to_ecef(X_i, julianDate)
R = local_rot_z(local_greenwich_sidereal_angle(julianDate));
X_ecef = R' * X_i;
end

function X_i = local_ecef_to_eci(X_ecef, julianDate)
R = local_rot_z(local_greenwich_sidereal_angle(julianDate));
X_i = R * X_ecef;
end

function theta = local_greenwich_sidereal_angle(julianDate)
%LOCAL_GREENWICH_SIDEREAL_ANGLE 计算格林尼治平恒星时角。
T = (julianDate - 2451545.0) / 36525.0;
thetaDeg = 280.46061837 ...
    + 360.98564736629 * (julianDate - 2451545.0) ...
    + 0.000387933 * T^2 ...
    - (T^3) / 38710000.0;
theta = deg2rad(mod(thetaDeg, 360.0));
end

function [latDeg, lonDeg, altM] = local_ecef_to_geodetic_wgs84(X_ecef)
%LOCAL_ECEF_TO_GEODETIC_WGS84 ECEF 到 WGS84 大地坐标的近似转换。
a = 6378137.0;
f = 1.0 / 298.257223563;
e2 = f * (2.0 - f);

x = X_ecef(1);
y = X_ecef(2);
z = X_ecef(3);
lon = atan2(y, x);
p = hypot(x, y);
lat = atan2(z, max(p, eps) * (1.0 - e2));

for k = 1:5
    sinLat = sin(lat);
    N = a / sqrt(1.0 - e2 * sinLat^2);
    alt = p / max(cos(lat), eps) - N;
    lat = atan2(z, p * (1.0 - e2 * N / max(N + alt, eps)));
end

sinLat = sin(lat);
N = a / sqrt(1.0 - e2 * sinLat^2);
alt = p / max(cos(lat), eps) - N;

latDeg = rad2deg(lat);
lonDeg = rad2deg(lon);
altM = alt;
end

function v_ecef = local_ned_to_ecef(v_ned, latDeg, lonDeg)
%LOCAL_NED_TO_ECEF NED 向量转 ECEF 向量。
lat = deg2rad(latDeg);
lon = deg2rad(lonDeg);
C_ecef_to_ned = [ ...
    -sin(lat) * cos(lon), -sin(lat) * sin(lon),  cos(lat); ...
    -sin(lon),             cos(lon),             0.0; ...
    -cos(lat) * cos(lon), -cos(lat) * sin(lon), -sin(lat)];
v_ecef = C_ecef_to_ned' * v_ned;
end

function decimalYear = local_decimal_year_from_jd(julianDate)
%LOCAL_DECIMAL_YEAR_FROM_JD 由儒略日近似计算十进制年份。
[yearValue, monthValue, dayValue, fracDay] = local_jd_to_calendar(julianDate);
monthDays = [31, 28 + local_is_leap_year(yearValue), 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
dayOfYear = dayValue + sum(monthDays(1:max(monthValue - 1, 0))) + fracDay;
yearLength = 365 + local_is_leap_year(yearValue);
decimalYear = yearValue + (dayOfYear - 1.0) / yearLength;
end

function [yearValue, monthValue, dayValue, fracDay] = local_jd_to_calendar(julianDate)
%LOCAL_JD_TO_CALENDAR 儒略日转公历日期。
Z = floor(julianDate + 0.5);
F = julianDate + 0.5 - Z;

if Z < 2299161
    A = Z;
else
    alpha = floor((Z - 1867216.25) / 36524.25);
    A = Z + 1 + alpha - floor(alpha / 4);
end

B = A + 1524;
C = floor((B - 122.1) / 365.25);
D = floor(365.25 * C);
E = floor((B - D) / 30.6001);

dayReal = B - D - floor(30.6001 * E) + F;
dayValue = floor(dayReal);
fracDay = dayReal - dayValue;

if E < 14
    monthValue = E - 1;
else
    monthValue = E - 13;
end

if monthValue > 2
    yearValue = C - 4716;
else
    yearValue = C - 4715;
end
end

function tf = local_is_leap_year(yearValue)
%LOCAL_IS_LEAP_YEAR 判断闰年。
tf = mod(yearValue, 4) == 0 && (mod(yearValue, 100) ~= 0 || mod(yearValue, 400) == 0);
end

function Rz = local_rot_z(theta)
Rz = [cos(theta), -sin(theta), 0.0; ...
      sin(theta),  cos(theta), 0.0; ...
      0.0,         0.0,        1.0];
end

function modelName = local_get_model_name(env, fieldName, defaultValue)
if isfield(env, fieldName) && ~isempty(env.(fieldName))
    modelName = env.(fieldName);
else
    modelName = defaultValue;
end
end