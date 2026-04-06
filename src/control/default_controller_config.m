function controllerConfig = default_controller_config()
%DEFAULT_CONTROLLER_CONFIG 默认姿态控制器配置。
%
% 当前控制架构采用串级结构：
%   1. 姿态外环：根据姿态误差生成目标角速度。
%   2. 角速度内环：根据角速度误差生成目标力矩。
%
% 控制模式：
%   - detumble      : B-dot 去旋
%   - coarse-point  : 大姿态误差时的粗指向控制
%   - fine-point    : 小姿态误差时的精指向控制
%   - momentum-dump : 飞轮动量卸载控制

controllerConfig.defaultMode = 'auto';
controllerConfig.activeMode = 'auto';

controllerConfig.switch.detumbleRateThresholdRadS = deg2rad(2.0);
controllerConfig.switch.coarseAttitudeErrorRad = deg2rad(15.0);
controllerConfig.switch.fineAttitudeErrorRad = deg2rad(3.0);
controllerConfig.switch.momentumDumpRatio = 0.80;
controllerConfig.switch.minMagneticFieldNormT = 1e-8;

controllerConfig.detumble.enabled = true;
controllerConfig.detumble.kBdot = 8e3;
controllerConfig.detumble.maxTorqueNm = [2e-3; 2e-3; 2e-3];

% 姿态外环参数：输出目标角速度。
controllerConfig.attitudeOuter.enabled = true;
controllerConfig.attitudeOuter.kqCoarse = [0.18; 0.18; 0.18];
controllerConfig.attitudeOuter.kqFine = [0.06; 0.06; 0.06];
controllerConfig.attitudeOuter.maxOmegaCmdRadS = deg2rad([8.0; 8.0; 8.0]);

% 角速度内环参数：输出目标力矩。
controllerConfig.rateInner.enabled = true;
controllerConfig.rateInner.kwCoarse = [0.10; 0.10; 0.10];
controllerConfig.rateInner.kwFine = [0.04; 0.04; 0.04];
controllerConfig.rateInner.maxTorqueNm = [0.020; 0.020; 0.020];

controllerConfig.momentumDump.enabled = true;
controllerConfig.momentumDump.kDump = 5e-2;
controllerConfig.momentumDump.maxDumpTorqueNm = [1.5e-3; 1.5e-3; 1.5e-3];
end