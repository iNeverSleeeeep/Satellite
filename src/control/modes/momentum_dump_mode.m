function [torqueCmd_b, controlMeta] = momentum_dump_mode(controllerInput, dumpConfig)
%MOMENTUM_DUMP_MODE 飞轮动量卸载控制模式。

wheelMomentumNms = controllerInput.wheelMomentumNms(:);
torqueCmd_b = -dumpConfig.kDump .* wheelMomentumNms;
torqueCmd_b = min(max(torqueCmd_b, -dumpConfig.maxDumpTorqueNm(:)), dumpConfig.maxDumpTorqueNm(:));

controlMeta.wheelMomentumNms = wheelMomentumNms;
end