function env = environment_config()
%ENVIRONMENT_CONFIG 环境模型的基础开关与常数配置。
%
% 说明:
%   本配置文件除了保存环境常数外，也提供参考方向模型的切换开关。
%   当前支持三类参考模型:
%   1. simple
%      用于原型验证，结构最简。
%   2. engineering
%      用于更正式的工程验证，比 simple 更接近真实物理过程。
%   3. high_fidelity
%      用于高保真模式，优先调用 MATLAB/Aerospace Toolbox 或外部函数句柄。
%      该模式不会悄悄退化为近似模型；若缺少依赖会直接报错。

env.useJ2 = true;
env.useAtmosphericDrag = false;
env.useSolarRadiationPressure = true;
env.useMagneticField = false;

env.earthMuM3S2 = 3.986004418e14;
env.earthRadiusM = 6378137.0;
env.earthJ2 = 1.082635854e-3;
env.earthRotationRadS = 7.292115e-5;
env.atmosphereScaleHeightM = 8500.0;
env.referenceDensityKgM3 = 1.225;
env.referenceAltitudeM = 0.0;
env.solarPressureNPM2 = 4.56e-6;

env.sunModel = 'engineering';
env.magneticModel = 'engineering';

% simple 太阳模型参数。
env.sunAngularRateRadS = 1.9910639e-7;
env.sunVector0_i = [1.0; 0.0; 0.2];

% 绝对历元。表示仿真 t = 0 对应的儒略日。
env.referenceEpochJulianDate = 2451545.0;

% simple 地磁模型参数。
env.earthMagneticDipoleAm2 = 7.94e22;
env.earthDipoleAxis_i = [0.0; 0.0; 1.0];

% engineering 地磁模型参数。
env.earthDipoleAxis_e = local_unit_vector([0.0656; -0.1860; 0.9804]);
env.earthRotationAtEpochRad = 0.0;

% high_fidelity 模式配置。
% sunFcn / magneticFcn 若给出，则优先调用用户提供的高保真函数。
% 未提供时，将尝试调用 MATLAB/Aerospace Toolbox 的权威接口。
env.highFidelity.sunFcn = [];
env.highFidelity.magneticFcn = [];
env.highFidelity.ephemerisModel = '421';
env.highFidelity.magneticModel = 'igrf';
end