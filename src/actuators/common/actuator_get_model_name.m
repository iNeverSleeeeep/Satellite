function modelName = actuator_get_model_name(config, defaultName)
%ACTUATOR_GET_MODEL_NAME 获取执行器模型模式名称。
if nargin < 2 || isempty(defaultName)
    defaultName = 'ideal';
end

modelName = defaultName;
if isfield(config, 'model') && ~isempty(config.model)
    modelName = char(lower(string(config.model)));
end
end