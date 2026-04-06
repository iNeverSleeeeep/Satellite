function [controllerOutput, controllerState] = controller_manager_step(controllerInput, controllerConfig, controllerState)
%CONTROLLER_MANAGER_STEP 控制器模式管理与单步控制入口。

if nargin < 3 || isempty(controllerState)
    controllerState = init_controller_state(controllerConfig);
end

modeName = select_controller_mode(controllerInput, controllerConfig);

switch modeName
    case 'detumble'
        [torqueCmd_b, modeState, controlMeta] = detumble_mode(controllerInput, controllerConfig.detumble, controllerState.detumble);
        controllerState.detumble = modeState;
    case 'momentum-dump'
        [torqueCmd_b, controlMeta] = momentum_dump_mode(controllerInput, controllerConfig.momentumDump);
    case 'fine-point'
        [torqueCmd_b, controlMeta] = attitude_cascade_mode(controllerInput, controllerConfig, 'fine');
    otherwise
        [torqueCmd_b, controlMeta] = attitude_cascade_mode(controllerInput, controllerConfig, 'coarse');
        modeName = 'coarse-point';
end

controllerState.activeMode = modeName;
controllerOutput.mode = modeName;
controllerOutput.commandedTorque_b = torqueCmd_b;
controllerOutput.controlMeta = controlMeta;
end