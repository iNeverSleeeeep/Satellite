function allocation = allocate_actuator_commands(torqueCmd_b, selectedActuator, actuatorConfig, actuatorContext)
%ALLOCATE_ACTUATOR_COMMANDS 将总力矩指令分解为三类执行器各自的命令。
%
% 核心思想：
%   这里采用“主执行器优先 + 剩余力矩逐级补偿”的分配方式。
%   不做二次规划或最优控制分配，而是追求：
%   1. 结构清楚；
%   2. 易于调试；
%   3. 后续方便替换成更复杂的优化分配器。
%
% 算法步骤：
%   1. 根据 selectedActuator 决定分配优先顺序。
%   2. 让优先级最高的执行器先尽可能承担力矩。
%   3. 计算剩余 residualTorque_b = torqueCmd_b - command_b。
%   4. 下一类执行器继续承担剩余部分，直到所有执行器都尝试过。
%
% 特殊处理：
%   - 飞轮、推力器：按各轴最大力矩做分量饱和。
%   - 磁力矩器：先把力矩投影到“垂直磁场平面”，因为平行于磁场的力矩无法实现。
%
% 输出：
%   allocation.reactionWheel.commandedTorque_b
%   allocation.magnetorquer.commandedTorque_b
%   allocation.thruster.commandedTorque_b
%   allocation.unallocatedTorque_b
%       所有执行器都尝试后仍无法实现的剩余力矩，可用于分析控制能力缺口。

torqueCmd_b = torqueCmd_b(:);
allocation.selectedActuator = selectedActuator;
allocation.requestedTorque_b = torqueCmd_b;
allocation.reactionWheel.commandedTorque_b = zeros(3, 1);
allocation.magnetorquer.commandedTorque_b = zeros(3, 1);
allocation.thruster.commandedTorque_b = zeros(3, 1);

priority = local_priority(selectedActuator);
residualTorque_b = torqueCmd_b;

for k = 1:numel(priority)
    actuatorName = priority{k};
    switch actuatorName
        case 'reaction-wheel'
            if ~actuatorConfig.reactionWheel.enabled
                continue;
            end

            % 飞轮按轴独立饱和，代表每轴电机都有最大连续控制力矩限制。
            command_b = local_limit_vector(residualTorque_b, actuatorConfig.reactionWheel.maxTorqueNm);
            allocation.reactionWheel.commandedTorque_b = command_b;

        case 'magnetorquer'
            if ~actuatorConfig.magnetorquer.enabled
                continue;
            end

            % 磁力矩器只能实现垂直于 B 的力矩，这里先做几何投影，
            % 再根据最大磁偶极矩限制反算得到可实现力矩。
            command_b = local_project_magnetic_torque(residualTorque_b, actuatorContext, actuatorConfig);
            allocation.magnetorquer.commandedTorque_b = command_b;

        case 'thruster'
            if ~actuatorConfig.thruster.enabled
                continue;
            end

            % 推力器这里用等效控制力矩近似，同样按轴做最大力矩限制。
            command_b = local_limit_vector(residualTorque_b, actuatorConfig.thruster.maxTorqueNm);
            allocation.thruster.commandedTorque_b = command_b;

        otherwise
            continue;
    end

    % 把本执行器已经承担的部分从总需求中扣掉，剩余部分交给下一类执行器。
    residualTorque_b = residualTorque_b - command_b;
end

allocation.unallocatedTorque_b = residualTorque_b;
end

function priority = local_priority(selectedActuator)
%LOCAL_PRIORITY 根据主执行器给出分配顺序。
%
% 说明：
%   选中的主执行器先承担主要任务，其他执行器只负责补剩余项。
%   这样做可以让“自动模式选择”真正反映到分配行为上。
switch char(lower(string(selectedActuator)))
    case 'magnetorquer'
        priority = {'magnetorquer', 'reaction-wheel', 'thruster'};
    case 'thruster'
        priority = {'thruster', 'reaction-wheel', 'magnetorquer'};
    otherwise
        priority = {'reaction-wheel', 'magnetorquer', 'thruster'};
end
end

function limitedVector = local_limit_vector(vectorIn, vectorLimit)
%LOCAL_LIMIT_VECTOR 对向量按分量做对称饱和。
limitedVector = min(max(vectorIn(:), -abs(vectorLimit(:))), abs(vectorLimit(:)));
end

function projectedTorque_b = local_project_magnetic_torque(torqueCmd_b, actuatorContext, actuatorConfig)
%LOCAL_PROJECT_MAGNETIC_TORQUE 计算磁力矩器在当前磁场下可实现的力矩。
%
% 几何原理：
%   磁力矩器输出满足 tau = m x B。
%   因为叉乘结果一定垂直于 B，所以：
%   - tau 在 B 方向上的分量永远无法产生；
%   - 只有垂直于 B 的那部分力矩才是“可控”的。
%
% 计算流程：
%   1. 从目标力矩中减去平行于 B 的分量，得到可实现投影 projectedTorque_b。
%   2. 由关系式反算等效磁偶极矩：
%          m = (B x tau) / |B|^2
%   3. 再根据最大磁偶极矩约束进行饱和。
%   4. 用饱和后的 m 重新计算最终可输出力矩。
projectedTorque_b = zeros(3, 1);

if ~isfield(actuatorContext, 'magneticField_b') || isempty(actuatorContext.magneticField_b)
    return;
end

magneticField_b = actuatorContext.magneticField_b(:);
magNorm = norm(magneticField_b);
if magNorm <= actuatorConfig.magnetorquer.minMagneticFieldNormT
    return;
end

% 第一步：剔除平行于磁场方向的不可实现分量。
bHat = magneticField_b / magNorm;
projectedTorque_b = torqueCmd_b(:) - bHat * dot(bHat, torqueCmd_b(:));

% 第二步：由目标力矩反推所需磁偶极矩。
maxDipole = actuatorConfig.magnetorquer.maxDipoleAm2(:);
equivalentDipole = cross(magneticField_b, projectedTorque_b) / (magNorm^2);

% 第三步：磁偶极矩受硬件能力限制。
equivalentDipole = local_limit_vector(equivalentDipole, maxDipole);

% 第四步：用受限后的磁偶极矩重新计算真正能输出的力矩。
projectedTorque_b = cross(equivalentDipole, magneticField_b);
end