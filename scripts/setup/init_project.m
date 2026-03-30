function init_project()
%INIT_PROJECT Load baseline configuration for the satellite simulation.

setup_paths();
create_bus_objects();

env = environment_config();
simConfig = default_sim_config();
spacecraft = spacecraft_params();

assignin('base', 'env', env);
assignin('base', 'simConfig', simConfig);
assignin('base', 'spacecraft', spacecraft);
disp('Satellite simulation project initialized.');
end
