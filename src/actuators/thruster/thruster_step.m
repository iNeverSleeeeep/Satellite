function [thrusterOutput, thrusterState] = thruster_step(commandedTorque_b, thrusterConfig, thrusterState, dt)
%THRUSTER_STEP 推力器统一入口。
modelName = thruster_get_model_name(thrusterConfig);
if strcmp(modelName, 'physics')
    [thrusterOutput, thrusterState] = thruster_physics(commandedTorque_b, thrusterConfig, thrusterState, dt);
else
    [thrusterOutput, thrusterState] = thruster_ideal(commandedTorque_b, thrusterConfig, thrusterState, dt);
end
end