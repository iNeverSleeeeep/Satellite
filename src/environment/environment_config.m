function env = environment_config()
%ENVIRONMENT_CONFIG 环境模型的基础开关与常数配置。

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
env.sunAngularRateRadS = 1.9910639e-7;
env.sunVector0_i = [1.0; 0.0; 0.2];
env.earthMagneticDipoleT = 7.94e22;
env.earthDipoleAxis_i = [0.0; 0.0; 1.0];
end
