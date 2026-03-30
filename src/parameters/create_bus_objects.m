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
end
