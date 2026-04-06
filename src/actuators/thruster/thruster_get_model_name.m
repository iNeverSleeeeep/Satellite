function modelName = thruster_get_model_name(thrusterConfig)
%THRUSTER_GET_MODEL_NAME 获取推力器模型模式名称。
modelName = actuator_get_model_name(thrusterConfig, 'ideal');
end