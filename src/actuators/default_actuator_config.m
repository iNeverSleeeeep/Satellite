function actuatorConfig = default_actuator_config()
%DEFAULT_ACTUATOR_CONFIG 默认执行器配置。
%
% 这一版把执行器框架升级为“双模式”：
%   1. ideal   : 面向控制分配验证的等效模型，计算快、结构简单。
%   2. physics : 更接近星上执行链路的物理模型，用于评估真实可实现性。
%
% 当前实现策略：
%   - 飞轮：已提供 ideal 与 physics 两种模型。
%   - 磁力矩器：已提供 ideal 与 physics 两种模型。
%   - 推力器：先保留 ideal 行为，同时补齐 physics 模式开关和参数骨架。

actuatorConfig.defaultSelector = 'auto';

% 飞轮：适合连续小力矩精细控制。
actuatorConfig.reactionWheel.enabled = true;
actuatorConfig.reactionWheel.model = 'physics';
actuatorConfig.reactionWheel.maxTorqueNm = [5e-3; 5e-3; 5e-3];
actuatorConfig.reactionWheel.maxMomentumNms = [0.03; 0.03; 0.03];
actuatorConfig.reactionWheel.initialMomentumNms = [0.0; 0.0; 0.0];
actuatorConfig.reactionWheel.timeConstantS = 0.05;
actuatorConfig.reactionWheel.physics.wheelInertiaKgM2 = [1.5e-4; 1.5e-4; 1.5e-4];
actuatorConfig.reactionWheel.physics.maxWheelSpeedRadS = [6000; 6000; 6000] .* (2 * pi / 60);
actuatorConfig.reactionWheel.physics.initialWheelSpeedRadS = [0.0; 0.0; 0.0];
actuatorConfig.reactionWheel.physics.motorTorqueConstantNmPerA = [5e-3; 5e-3; 5e-3];
actuatorConfig.reactionWheel.physics.maxMotorCurrentA = [1.2; 1.2; 1.2];
actuatorConfig.reactionWheel.physics.viscousFrictionNmPerRadS = [2e-6; 2e-6; 2e-6];
actuatorConfig.reactionWheel.physics.coulombFrictionNm = [3e-5; 3e-5; 3e-5];
actuatorConfig.reactionWheel.physics.driveTimeConstantS = 0.02;

% 磁力矩器：适合飞轮卸载和低功耗控制。
actuatorConfig.magnetorquer.enabled = true;
actuatorConfig.magnetorquer.model = 'physics';
actuatorConfig.magnetorquer.maxDipoleAm2 = [0.25; 0.25; 0.25];
actuatorConfig.magnetorquer.timeConstantS = 0.10;
actuatorConfig.magnetorquer.minMagneticFieldNormT = 1e-8;
% 物理参数说明：
%   maxCoilCurrentA     : 线圈最大允许电流。
%   dipolePerAmpAm2     : 单位电流产生的磁偶极矩常数。
%   coilResistanceOhm   : 线圈电阻。
%   coilInductanceH     : 线圈电感。
%   driveVoltageV       : 驱动电源/驱动器可提供的最大电压。
%   currentControllerGain : 电流闭环的比例增益，用于把磁矩需求转成驱动电压。
actuatorConfig.magnetorquer.physics.maxCoilCurrentA = [0.20; 0.20; 0.20];
actuatorConfig.magnetorquer.physics.dipolePerAmpAm2 = [1.25; 1.25; 1.25];
actuatorConfig.magnetorquer.physics.coilResistanceOhm = [8.0; 8.0; 8.0];
actuatorConfig.magnetorquer.physics.coilInductanceH = [0.12; 0.12; 0.12];
actuatorConfig.magnetorquer.physics.driveVoltageV = [5.0; 5.0; 5.0];
actuatorConfig.magnetorquer.physics.currentControllerGain = [12.0; 12.0; 12.0];
actuatorConfig.magnetorquer.physics.coilTimeConstantS = 0.03;

% 推力器：适合大力矩机动。
actuatorConfig.thruster.enabled = true;
actuatorConfig.thruster.model = 'ideal';
actuatorConfig.thruster.maxTorqueNm = [0.05; 0.05; 0.05];
actuatorConfig.thruster.minPulseTorqueNm = [5e-3; 5e-3; 5e-3];
actuatorConfig.thruster.timeConstantS = 0.02;
actuatorConfig.thruster.physics.valveDelayS = 0.01;
actuatorConfig.thruster.physics.minOnTimeS = 0.02;
actuatorConfig.thruster.physics.maxForceN = [0.2; 0.2; 0.2];
actuatorConfig.thruster.physics.leverArmM = [0.25; 0.25; 0.25];

% 自动选择阈值。
actuatorConfig.auto.reactionWheelTorqueThresholdNm = 4e-3;
actuatorConfig.auto.thrusterTorqueThresholdNm = 1.2e-2;
actuatorConfig.auto.reactionWheelMomentumUnloadRatio = 0.80;
actuatorConfig.auto.magnetorquerAlignmentThreshold = 0.25;
end