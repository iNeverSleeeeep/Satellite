function [magnetorquerOutput, magnetorquerState] = magnetorquer_step(commandedTorque_b, magneticField_b, magnetorquerConfig, magnetorquerState, dt)
%MAGNETORQUER_STEP 磁力矩器统一入口。
modelName = magnetorquer_get_model_name(magnetorquerConfig);
if strcmp(modelName, 'physics')
    [magnetorquerOutput, magnetorquerState] = magnetorquer_physics(commandedTorque_b, magneticField_b, magnetorquerConfig, magnetorquerState, dt);
else
    [magnetorquerOutput, magnetorquerState] = magnetorquer_ideal(commandedTorque_b, magneticField_b, magnetorquerConfig, magnetorquerState, dt);
end
end