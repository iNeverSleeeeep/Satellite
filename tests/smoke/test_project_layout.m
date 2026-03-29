function test_project_layout()
%TEST_PROJECT_LAYOUT Basic smoke test for project bootstrap scripts.

setup_paths();
cfg = default_sim_config();

assert(isstruct(cfg), 'default_sim_config must return a struct.');
assert(isfield(cfg, 'stopTime'), 'Simulation config must define stopTime.');
end
