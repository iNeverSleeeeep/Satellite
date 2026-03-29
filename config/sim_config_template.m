function cfg = sim_config_template()
%SIM_CONFIG_TEMPLATE Template for scenario-specific overrides.

cfg = default_sim_config();
cfg.name = 'custom-case';
end
