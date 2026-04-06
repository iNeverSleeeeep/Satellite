function [actuatorOutput, actuatorState] = actuator_step(torqueCmd_b, actuatorConfig, actuatorState, dt, actuatorContext)
%ACTUATOR_STEP 执行器链路单步更新。
%
% 这是执行器子系统的统一入口，负责把“控制器给出的总力矩指令”转换成
% “三类执行器各自的命令和实际输出”。
%
% 总体流程：
%   1. 补齐上下文和初始状态；
%   2. 自动选择当前主执行器；
%   3. 将总力矩分配到飞轮、磁力矩器、推力器三类执行器；
%   4. 分别调用每类执行器模型，得到实际输出；
%   5. 合成总控制力矩并给出跟踪误差。
%
% 输入：
%   torqueCmd_b
%       控制器输出的机体系总力矩指令 [3x1]。
%   actuatorConfig
%       执行器配置参数，通常来自 default_actuator_config()。
%   actuatorState
%       执行器内部状态；首次调用可传空 []，函数会自动初始化。
%   dt
%       仿真步长。
%   actuatorContext
%       执行器上下文，可选字段包括：
%       - magneticField_b    : 机体系磁场，用于磁力矩器模型和自动选择逻辑。
%       - preferredActuator  : 外部强制指定主执行器。
%
% 输出：
%   actuatorOutput
%       聚合输出结构，包含：
%       - selectedActuator   : 当前主执行器
%       - allocation         : 力矩分配结果
%       - reactionWheel      : 飞轮模型输出
%       - magnetorquer       : 磁力矩器模型输出
%       - thruster           : 推力器模型输出
%       - netTorque_b        : 三类执行器合成后的实际总力矩
%       - trackingError_b    : 总力矩跟踪误差
%   actuatorState
%       更新后的内部状态，供下一步继续使用。

% 若未提供上下文，则补一个空结构，保证后续字段访问安全。
if nargin < 5 || isempty(actuatorContext)
    actuatorContext = struct();
end

% 若首次调用还没有状态，则在此创建初始状态。
if nargin < 3 || isempty(actuatorState)
    actuatorState = init_actuator_state(actuatorConfig);
end

% 磁力矩器相关逻辑都依赖 magneticField_b。
% 若外部没有给，就默认没有可用磁场，等价于磁力矩器当前无法工作。
if ~isfield(actuatorContext, 'magneticField_b') || isempty(actuatorContext.magneticField_b)
    actuatorContext.magneticField_b = zeros(3, 1);
end

% 第一步：根据当前指令幅值、飞轮状态和地磁条件，选择主执行器。
selectedActuator = select_actuator_mode(torqueCmd_b, actuatorConfig, actuatorState, actuatorContext);

% 第二步：将总力矩分解成三类执行器各自承担的部分。
allocation = allocate_actuator_commands(torqueCmd_b, selectedActuator, actuatorConfig, actuatorContext);

% 第三步：分别经过各自执行器模型，得到“实际”而非理想的输出。
[reactionWheelOutput, actuatorState.reactionWheel] = simulate_reaction_wheel_actuator( ...
    allocation.reactionWheel.commandedTorque_b, actuatorConfig.reactionWheel, actuatorState.reactionWheel, dt);
[magnetorquerOutput, actuatorState.magnetorquer] = simulate_magnetorquer_actuator( ...
    allocation.magnetorquer.commandedTorque_b, actuatorContext.magneticField_b, actuatorConfig.magnetorquer, actuatorState.magnetorquer, dt);
[thrusterOutput, actuatorState.thruster] = simulate_thruster_actuator( ...
    allocation.thruster.commandedTorque_b, actuatorConfig.thruster, actuatorState.thruster, dt);

% 保存本拍主执行器，便于诊断和模式追踪。
actuatorState.selectedActuator = selectedActuator;

% 汇总输出，供外层动力学或日志记录模块使用。
actuatorOutput.selectedActuator = selectedActuator;
actuatorOutput.requestedTorque_b = torqueCmd_b(:);
actuatorOutput.allocation = allocation;
actuatorOutput.reactionWheel = reactionWheelOutput;
actuatorOutput.magnetorquer = magnetorquerOutput;
actuatorOutput.thruster = thrusterOutput;

% 合成三类执行器的实际力矩输出，得到真正作用在航天器上的控制力矩。
actuatorOutput.netTorque_b = reactionWheelOutput.actualTorque_b ...
    + magnetorquerOutput.actualTorque_b ...
    + thrusterOutput.actualTorque_b;

% 跟踪误差反映“想要的总力矩”和“实际总力矩”之间的差值。
% 若误差较大，通常说明：
%   - 执行器饱和；
%   - 磁力矩器方向受限；
%   - 推力器量化；
%   - 一阶动态导致瞬时跟踪不完全。
actuatorOutput.trackingError_b = torqueCmd_b(:) - actuatorOutput.netTorque_b;
end