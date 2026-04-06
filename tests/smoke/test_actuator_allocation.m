function test_actuator_allocation()
%TEST_ACTUATOR_ALLOCATION Smoke test for actuator selection and allocation.

setup_paths();

gnc = gnc_overview();
cfg = gnc.actuators;
cfg.reactionWheel.timeConstantS = 0.0;
cfg.magnetorquer.timeConstantS = 0.0;
cfg.thruster.timeConstantS = 0.0;
cfg.reactionWheel.physics.driveTimeConstantS = 0.0;

state = init_actuator_state(cfg);
dt = 0.1;

[smallOutput, state] = actuator_step([2e-3; -1e-3; 1e-3], cfg, state, dt, struct());
assert(strcmp(smallOutput.selectedActuator, 'reaction-wheel'), 'Small torque commands should default to reaction wheel.');
assert(norm(smallOutput.reactionWheel.actualTorque_b) > 0.0, 'Reaction wheel should produce non-zero torque for small commands.');
assert(strcmp(smallOutput.reactionWheel.model, cfg.reactionWheel.model), 'Reaction wheel model selection should be reflected in the output.');

[largeOutput, state] = actuator_step([2e-2; 0.0; 0.0], cfg, state, dt, struct());
assert(strcmp(largeOutput.selectedActuator, 'thruster'), 'Large torque commands should default to thruster.');
assert(abs(largeOutput.thruster.actualTorque_b(1)) >= cfg.thruster.minPulseTorqueNm(1), 'Thruster should produce quantized output torque.');

state.reactionWheel.momentumNms = 0.95 .* cfg.reactionWheel.maxMomentumNms;
magneticField_b = [0.0; 0.0; 3.5e-5];
[magOutput, state] = actuator_step([3e-3; 0.0; 0.0], cfg, state, dt, struct('magneticField_b', magneticField_b));
assert(strcmp(magOutput.selectedActuator, 'magnetorquer'), 'High wheel momentum with usable magnetic field should prefer magnetorquer.');
assert(strcmp(magOutput.magnetorquer.model, cfg.magnetorquer.model), 'Magnetorquer model selection should be reflected in the output.');
assert(abs(dot(magOutput.magnetorquer.actualTorque_b, magneticField_b)) < 1e-12, 'Magnetorquer output torque must stay perpendicular to the magnetic field.');
assert(any(abs(magOutput.magnetorquer.coilCurrentA) > 0), 'Magnetorquer physics mode should produce non-zero coil current.');
assert(norm(magOutput.netTorque_b) > 0.0, 'Actuator suite should produce non-zero net torque.');

cfgIdeal = cfg;
cfgIdeal.reactionWheel.model = 'ideal';
cfgIdeal.magnetorquer.model = 'ideal';
stateIdeal = init_actuator_state(cfgIdeal);
[idealOutput, ~] = actuator_step([1e-3; 0; 0], cfgIdeal, stateIdeal, dt, struct('magneticField_b', magneticField_b));
assert(strcmp(idealOutput.reactionWheel.model, 'ideal'), 'Reaction wheel ideal mode should remain available.');
assert(strcmp(idealOutput.magnetorquer.model, 'ideal'), 'Magnetorquer ideal mode should remain available.');
end