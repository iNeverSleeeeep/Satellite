function rho = calc_atmospheric_density(X_i, env)
%CALC_ATMOSPHERIC_DENSITY 简化指数大气模型。
%
%   计算公式：
%   h   = |X_i| - R_e
%   rho = rho0 * exp(-(h - h0) / H)
%   其中 R_e 为地球平均半径，rho0 为参考密度，h0 为参考高度，
%   H 为大气尺度高度。
%
%   本函数只用于提供一个简单、连续的密度近似，适合前期工程联调。

altitude = max(norm(X_i) - env.earthRadiusM, 0.0);
rho = env.referenceDensityKgM3 * exp( ...
    -(altitude - env.referenceAltitudeM) / env.atmosphereScaleHeightM);
end
