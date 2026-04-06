function test_controller_manager()
%TEST_CONTROLLER_MANAGER Multi-controller and mode-switch smoke test.
%
% 本测试直接使用新的控制管理入口：
%   manager/controller_manager_step
% 不再通过旧的兼容包装层。

setup_paths();

gnc = gnc_overview();
cfg = gnc.controller;
state = init_controller_state(cfg);

baseInput.q_bi = [1; 0; 0; 0];
baseInput.q_ref_bi = [1; 0; 0; 0];
baseInput.omega_ref_b = [0; 0; 0];
baseInput.dt = 0.1;
baseInput.magneticField_b = [2e-5; -1e-5; 3e-5];
baseInput.wheelMomentumNms = [0; 0; 0];
baseInput.wheelMomentumMaxNms = [0.03; 0.03; 0.03];

inputDetumble = baseInput;
inputDetumble.omega_b = deg2rad([5; 0; 0]);
[outDetumble, state] = controller_manager_step(inputDetumble, cfg, state);
assert(strcmp(outDetumble.mode, 'detumble'), 'High body rate should select detumble mode.');

inputDump = baseInput;
inputDump.omega_b = [0; 0; 0];
inputDump.wheelMomentumNms = 0.95 .* inputDump.wheelMomentumMaxNms;
[outDump, state] = controller_manager_step(inputDump, cfg, state);
assert(strcmp(outDump.mode, 'momentum-dump'), 'High wheel momentum should select momentum dump mode.');

inputCoarse = baseInput;
inputCoarse.omega_b = [0; 0; 0];
inputCoarse.q_bi = normalize_q([cos(pi/10); sin(pi/10); 0; 0]);
[outCoarse, state] = controller_manager_step(inputCoarse, cfg, state);
assert(strcmp(outCoarse.mode, 'coarse-point'), 'Large attitude error should select coarse pointing mode.');
assert(all(size(outCoarse.controlMeta.omegaCmd_b) == [3, 1]), 'Outer loop should output a 3x1 rate command.');
assert(all(size(outCoarse.commandedTorque_b) == [3, 1]), 'Rate loop should output a 3x1 torque command.');

inputFine = baseInput;
inputFine.omega_b = [0; 0; 0];
inputFine.q_bi = normalize_q([cos(deg2rad(1)/2); sin(deg2rad(1)/2); 0; 0]);
[outFine, state] = controller_manager_step(inputFine, cfg, state);
assert(strcmp(outFine.mode, 'fine-point'), 'Small attitude error should select fine pointing mode.');
assert(all(size(outFine.commandedTorque_b) == [3, 1]), 'Controller output torque must be a 3x1 vector.');
end