function create_bus_objects()
%CREATE_BUS_OBJECTS 定义项目中的 Simulink Bus 对象。

stateBusElems(1) = Simulink.BusElement;
stateBusElems(1).Name = 'X_i';
stateBusElems(1).Dimensions = [3 1];

stateBusElems(2) = Simulink.BusElement;
stateBusElems(2).Name = 'V_i';
stateBusElems(2).Dimensions = [3 1];

stateBusElems(3) = Simulink.BusElement;
stateBusElems(3).Name = 'A_i';
stateBusElems(3).Dimensions = [3 1];

stateBusElems(4) = Simulink.BusElement;
stateBusElems(4).Name = 'q_bi';
stateBusElems(4).Dimensions = [4 1];

stateBusElems(5) = Simulink.BusElement;
stateBusElems(5).Name = 'omega_b';
stateBusElems(5).Dimensions = [3 1];

stateBusElems(6) = Simulink.BusElement;
stateBusElems(6).Name = 'omega_dot_b';
stateBusElems(6).Dimensions = [3 1];

stateBusElems(7) = Simulink.BusElement;
stateBusElems(7).Name = 'DCM_bf';
stateBusElems(7).Dimensions = [3 3];

stateBusElems(8) = Simulink.BusElement;
stateBusElems(8).Name = 'DCM_if';
stateBusElems(8).Dimensions = [3 3];

stateBusElems(9) = Simulink.BusElement;
stateBusElems(9).Name = 'DCM_bi';
stateBusElems(9).Dimensions = [3 3];

stateBusElems(10) = Simulink.BusElement;
stateBusElems(10).Name = 'Forces_b';
stateBusElems(10).Dimensions = [3 1];

stateBusElems(11) = Simulink.BusElement;
stateBusElems(11).Name = 'Moments_b';
stateBusElems(11).Dimensions = [3 1];

StatesBus = Simulink.Bus;
StatesBus.Elements = stateBusElems;
assignin('base', 'StatesBus', StatesBus);

%% MissionBus
missionBusElems(1) = Simulink.BusElement;
missionBusElems(1).Name = 'q_ref';
missionBusElems(1).Dimensions = [4 1];

missionBusElems(2) = Simulink.BusElement;
missionBusElems(2).Name = 'omega_ref';
missionBusElems(2).Dimensions = [3 1];

MissionBus = Simulink.Bus;
MissionBus.Elements = missionBusElems;
assignin('base', 'MissionBus', MissionBus);

%% SensorsBus
sensorBusElems(1) = Simulink.BusElement;
sensorBusElems(1).Name = 'q_meas';
sensorBusElems(1).Dimensions = [4 1];

sensorBusElems(2) = Simulink.BusElement;
sensorBusElems(2).Name = 'omega_meas';
sensorBusElems(2).Dimensions = [3 1];

SensorsBus = Simulink.Bus;
SensorsBus.Elements = sensorBusElems;
assignin('base', 'SensorsBus', SensorsBus);

%% EnvBus
envBusElems(1) = Simulink.BusElement;
envBusElems(1).Name = 'useJ2';
envBusElems(1).Dimensions = 1;
envBusElems(1).DataType = 'boolean';

envBusElems(2) = Simulink.BusElement;
envBusElems(2).Name = 'useAtmosphericDrag';
envBusElems(2).Dimensions = 1;
envBusElems(2).DataType = 'boolean';

envBusElems(3) = Simulink.BusElement;
envBusElems(3).Name = 'useSolarRadiationPressure';
envBusElems(3).Dimensions = 1;
envBusElems(3).DataType = 'boolean';

envBusElems(4) = Simulink.BusElement;
envBusElems(4).Name = 'useMagneticField';
envBusElems(4).Dimensions = 1;
envBusElems(4).DataType = 'boolean';

envBusElems(5) = Simulink.BusElement;
envBusElems(5).Name = 'earthMuM3S2';
envBusElems(5).Dimensions = 1;

envBusElems(6) = Simulink.BusElement;
envBusElems(6).Name = 'earthRadiusM';
envBusElems(6).Dimensions = 1;

envBusElems(7) = Simulink.BusElement;
envBusElems(7).Name = 'earthJ2';
envBusElems(7).Dimensions = 1;

envBusElems(8) = Simulink.BusElement;
envBusElems(8).Name = 'earthRotationRadS';
envBusElems(8).Dimensions = 1;

envBusElems(9) = Simulink.BusElement;
envBusElems(9).Name = 'atmosphereScaleHeightM';
envBusElems(9).Dimensions = 1;

envBusElems(10) = Simulink.BusElement;
envBusElems(10).Name = 'referenceDensityKgM3';
envBusElems(10).Dimensions = 1;

envBusElems(11) = Simulink.BusElement;
envBusElems(11).Name = 'referenceAltitudeM';
envBusElems(11).Dimensions = 1;

envBusElems(12) = Simulink.BusElement;
envBusElems(12).Name = 'solarPressureNPM2';
envBusElems(12).Dimensions = 1;

envBusElems(13) = Simulink.BusElement;
envBusElems(13).Name = 'sunAngularRateRadS';
envBusElems(13).Dimensions = 1;

envBusElems(14) = Simulink.BusElement;
envBusElems(14).Name = 'sunVector0_i';
envBusElems(14).Dimensions = [3 1];

envBusElems(15) = Simulink.BusElement;
envBusElems(15).Name = 'earthMagneticDipoleT';
envBusElems(15).Dimensions = 1;

envBusElems(16) = Simulink.BusElement;
envBusElems(16).Name = 'earthDipoleAxis_i';
envBusElems(16).Dimensions = [3 1];

EnvBus = Simulink.Bus;
EnvBus.Elements = envBusElems;
assignin('base', 'EnvBus', EnvBus);

%% SpacecraftBus
spacecraftBusElems(1) = Simulink.BusElement;
spacecraftBusElems(1).Name = 'massKg';
spacecraftBusElems(1).Dimensions = 1;

spacecraftBusElems(2) = Simulink.BusElement;
spacecraftBusElems(2).Name = 'inertiaKgM2';
spacecraftBusElems(2).Dimensions = [3 3];

spacecraftBusElems(3) = Simulink.BusElement;
spacecraftBusElems(3).Name = 'centerOfMassM';
spacecraftBusElems(3).Dimensions = [3 1];

spacecraftBusElems(4) = Simulink.BusElement;
spacecraftBusElems(4).Name = 'maxTorqueNm';
spacecraftBusElems(4).Dimensions = [3 1];

spacecraftBusElems(5) = Simulink.BusElement;
spacecraftBusElems(5).Name = 'dragCoefficient';
spacecraftBusElems(5).Dimensions = 1;

spacecraftBusElems(6) = Simulink.BusElement;
spacecraftBusElems(6).Name = 'dragAreaM2';
spacecraftBusElems(6).Dimensions = 1;

spacecraftBusElems(7) = Simulink.BusElement;
spacecraftBusElems(7).Name = 'solarPressureCoefficient';
spacecraftBusElems(7).Dimensions = 1;

spacecraftBusElems(8) = Simulink.BusElement;
spacecraftBusElems(8).Name = 'solarAreaM2';
spacecraftBusElems(8).Dimensions = 1;

spacecraftBusElems(9) = Simulink.BusElement;
spacecraftBusElems(9).Name = 'centerOfPressureM';
spacecraftBusElems(9).Dimensions = [3 1];

spacecraftBusElems(10) = Simulink.BusElement;
spacecraftBusElems(10).Name = 'centerOfDragM';
spacecraftBusElems(10).Dimensions = [3 1];

spacecraftBusElems(11) = Simulink.BusElement;
spacecraftBusElems(11).Name = 'residualDipoleAm2';
spacecraftBusElems(11).Dimensions = [3 1];

SpacecraftBus = Simulink.Bus;
SpacecraftBus.Elements = spacecraftBusElems;
assignin('base', 'SpacecraftBus', SpacecraftBus);
end
