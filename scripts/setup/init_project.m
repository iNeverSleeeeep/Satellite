function [simConfig, spacecraft] = init_project()
%INIT_PROJECT Load baseline configuration for the satellite simulation.

setup_paths();

simConfig = default_sim_config();
spacecraft = spacecraft_params();
disp('Satellite simulation project initialized.');
end
