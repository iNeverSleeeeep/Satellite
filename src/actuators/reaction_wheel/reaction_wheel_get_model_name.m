function modelName = reaction_wheel_get_model_name(reactionWheelConfig)
%REACTION_WHEEL_GET_MODEL_NAME 获取飞轮模型模式名称。
modelName = actuator_get_model_name(reactionWheelConfig, 'ideal');
end