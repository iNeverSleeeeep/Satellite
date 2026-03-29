function simConfig = init_project()
%INIT_PROJECT Load baseline configuration for the satellite simulation.

setup_paths();

simConfig = default_sim_config();
disp('Satellite simulation project initialized.');
end
