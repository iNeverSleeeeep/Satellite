function test_project_layout()
%TEST_PROJECT_LAYOUT Basic smoke test for project bootstrap scripts.

setup_paths();
cfg = default_sim_config();
gnc = gnc_overview();

assert(isstruct(cfg), 'default_sim_config must return a struct.');
assert(isfield(cfg, 'stopTime'), 'Simulation config must define stopTime.');
assert(isfield(gnc, 'sensorSuite'), 'GNC config must define a sensor suite.');
assert(isfield(gnc, 'actuators'), 'GNC config must define actuator settings.');
assert(isfield(gnc, 'controller'), 'GNC config must define controller settings.');
assert(isfolder(fullfile(project_root(), 'src', 'sensors')), 'src/sensors folder must exist.');
assert(isfolder(fullfile(project_root(), 'src', 'observers')), 'src/observers folder must exist.');
assert(isfolder(fullfile(project_root(), 'src', 'actuators')), 'src/actuators folder must exist.');
assert(isfolder(fullfile(project_root(), 'src', 'control')), 'src/control folder must exist.');
end