function modelName = magnetorquer_get_model_name(magnetorquerConfig)
%MAGNETORQUER_GET_MODEL_NAME 获取磁力矩器模型模式名称。
modelName = actuator_get_model_name(magnetorquerConfig, 'ideal');
end