function selectedActuator = select_actuator_mode(torqueCmd_b, actuatorConfig, actuatorState, actuatorContext)
%SELECT_ACTUATOR_MODE 根据当前工况自动选择主执行器。
%
% 选择思路：
%   这是一个“规则驱动”的模式选择器，不做复杂优化，只做工程上容易解释的决策。
%
%   优先级逻辑如下：
%   1. 如果外部显式指定 preferredActuator，则直接服从外部指定。
%   2. 如果 defaultSelector 不是 'auto'，则固定输出该模式。
%   3. 自动模式下：
%      - 大力矩需求：优先推力器。
%      - 飞轮接近饱和且地磁条件合适：优先磁力矩器做卸载/辅助控制。
%      - 小力矩精细控制：优先飞轮。
%      - 其余情况按可用性降级选择。
%
% 输入：
%   torqueCmd_b
%       控制器输出的机体系总力矩指令 [3x1]。
%   actuatorConfig
%       执行器参数。
%   actuatorState
%       当前执行器状态，主要读取飞轮角动量判断是否接近饱和。
%   actuatorContext
%       上下文信息，可包含：
%       - preferredActuator : 外部强制指定主执行器。
%       - magneticField_b    : 当前机体系磁场向量。
%
% 输出：
%   selectedActuator
%       本拍主执行器名称字符串。

selectedActuator = actuatorConfig.defaultSelector;

% 外部强制指定优先级最高，适合任务模式切换或人工调试。
if isfield(actuatorContext, 'preferredActuator') && ~isempty(actuatorContext.preferredActuator)
    selectedActuator = char(lower(string(actuatorContext.preferredActuator)));
    return;
end

% 如果配置中已经固定了执行器模式，则无需继续自动判断。
if ~strcmpi(selectedActuator, 'auto')
    selectedActuator = char(lower(string(selectedActuator)));
    return;
end

% 力矩幅值用于判断当前更像“精细控制”还是“大机动”。
torqueAbs = abs(torqueCmd_b(:));
torqueNorm = norm(torqueCmd_b);

% 规则 1：只要任一轴指令超过大力矩阈值，就优先推力器。
% 这是因为推力器更适合承担大机动，能减轻飞轮饱和风险。
if actuatorConfig.thruster.enabled && any(torqueAbs >= actuatorConfig.auto.thrusterTorqueThresholdNm)
    selectedActuator = 'thruster';
    return;
end

% 读取磁场信息，用于判断磁力矩器当前是否“有方向可做”。
magneticField_b = local_get_magnetic_field(actuatorContext);
magNorm = norm(magneticField_b);
magAvailable = actuatorConfig.magnetorquer.enabled && magNorm > actuatorConfig.magnetorquer.minMagneticFieldNormT;

% 飞轮接近饱和的典型判据：当前角动量已达到上限的一定比例。
wheelMomentumAbs = abs(actuatorState.reactionWheel.momentumNms(:));
wheelUnloadThreshold = actuatorConfig.auto.reactionWheelMomentumUnloadRatio .* actuatorConfig.reactionWheel.maxMomentumNms(:);
wheelNearSaturation = actuatorConfig.reactionWheel.enabled && any(wheelMomentumAbs >= wheelUnloadThreshold);

% 规则 2：若飞轮接近饱和，并且磁场存在且方向可利用，则优先磁力矩器。
% 这里的“方向可利用”通过 torquePerpRatio 衡量：
%   - 先把目标力矩在垂直于磁场的平面内投影；
%   - 再看这个可实现部分占总需求的比例有多大。
% 如果比例过小，说明大部分需求方向无法由磁力矩器完成，不值得优先选它。
if magAvailable && wheelNearSaturation
    torquePerpRatio = norm(torqueCmd_b - magneticField_b * (dot(magneticField_b, torqueCmd_b) / magNorm^2)) / max(torqueNorm, eps);
    if torquePerpRatio >= actuatorConfig.auto.magnetorquerAlignmentThreshold
        selectedActuator = 'magnetorquer';
        return;
    end
end

% 规则 3：小力矩连续控制默认给飞轮，利于姿态稳定和精细控制。
if actuatorConfig.reactionWheel.enabled && all(torqueAbs <= actuatorConfig.auto.reactionWheelTorqueThresholdNm)
    selectedActuator = 'reaction-wheel';
    return;
end

% 规则 4：兜底降级逻辑。
% 优先飞轮，其次磁力矩器，再次推力器。
if actuatorConfig.reactionWheel.enabled
    selectedActuator = 'reaction-wheel';
elseif magAvailable
    selectedActuator = 'magnetorquer';
elseif actuatorConfig.thruster.enabled
    selectedActuator = 'thruster';
else
    selectedActuator = 'none';
end
end

function magneticField_b = local_get_magnetic_field(actuatorContext)
%LOCAL_GET_MAGNETIC_FIELD 读取机体系磁场，若缺省则返回零向量。
magneticField_b = zeros(3, 1);
if isfield(actuatorContext, 'magneticField_b') && ~isempty(actuatorContext.magneticField_b)
    magneticField_b = actuatorContext.magneticField_b(:);
end
end