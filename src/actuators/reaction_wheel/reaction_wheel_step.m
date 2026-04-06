function [reactionWheelOutput, reactionWheelState] = reaction_wheel_step(commandedTorque_b, reactionWheelConfig, reactionWheelState, dt)
%REACTION_WHEEL_STEP 飞轮执行器统一入口。
%
% 职责划分：
%   - 本文件只负责路由和组织流程；
%   - 理想模型和物理模型分别下沉到独立文件；
%   - 这样外层接口稳定，但内部结构更容易维护和测试。

commandedTorque_b = commandedTorque_b(:);
modelName = reaction_wheel_get_model_name(reactionWheelConfig);

switch modelName
    case 'physics'
        [reactionWheelOutput, reactionWheelState] = reaction_wheel_physics(commandedTorque_b, reactionWheelConfig, reactionWheelState, dt);
    otherwise
        [reactionWheelOutput, reactionWheelState] = reaction_wheel_ideal(commandedTorque_b, reactionWheelConfig, reactionWheelState, dt);
end
end