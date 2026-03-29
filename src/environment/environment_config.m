function env = environment_config()
%ENVIRONMENT_CONFIG Baseline environment model toggles.

env.useJ2 = true;
env.useAtmosphericDrag = false;
env.useSolarRadiationPressure = true;
env.useMagneticField = false;
end
