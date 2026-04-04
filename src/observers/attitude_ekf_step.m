function state = attitude_ekf_step(state, omega_b, dt, measurements, cfg)
%ATTITUDE_EKF_STEP 姿态扩展卡尔曼滤波器单步更新。
%
% 功能概述:
%   本函数完成一次完整的姿态 EKF 递推，包括:
%   1. 初始化判断
%   2. 状态预测
%   3. 量测更新
%
% 当前滤波器以四元数 q_bi 作为状态量，其中:
%   q_bi 表示“机体系 b 相对惯性系 i”的姿态四元数。
%
% 输入参数:
%   state        : 上一时刻滤波器状态结构体。
%                  常用字段包括:
%                    - q_bi           : 4x1 当前姿态四元数
%                    - P              : 4x4 状态协方差矩阵
%                    - isInitialized  : 是否已初始化
%                    - lastInnovation : 上一次量测残差
%   omega_b      : 3x1 机体系角速度，单位 rad/s。
%   dt           : 当前滤波步长，单位 s。
%   measurements : 量测结构体，通常由 measure_attitude_sensors 生成。
%                  其中包含:
%                    - measurements.sun.valid
%                    - measurements.sun.vector_b
%                    - measurements.mag.valid
%                    - measurements.mag.vector_b
%                    - measurements.reference.sun_i
%                    - measurements.reference.mag_i
%   cfg          : EKF 配置参数结构体。
%
% 输出参数:
%   state        : 更新后的滤波器状态结构体。
%
% 处理流程:
%   1. 若 state 为空或尚未初始化，则先尝试用太阳方向和磁场方向做一次
%      两矢量定姿，得到初始姿态 q0。
%   2. 用角速度进行姿态传播，得到预测姿态和预测协方差。
%   3. 用太阳敏感器和磁力计量测修正预测结果。
%
% 设计说明:
%   1. 该实现属于“直接四元数 EKF”的工程化版本，优点是结构直观、便于验证。
%   2. 量测模型直接使用“参考方向经姿态变换后应在机体系中的方向”进行比较。
%   3. 雅可比矩阵使用数值差分计算，牺牲了一些效率，但换来实现简单和更快迭代。
%   4. 若后续需要更强的数值一致性与实时性，可演进为误差状态 EKF 或 MEKF。

if nargin < 5
    cfg = attitude_ekf_config();
end

% 若滤波器尚未初始化，则优先使用两矢量定姿给出一个较合理的初值。
if isempty(state) || ~isfield(state, 'isInitialized') || ~state.isInitialized
    q0 = cfg.initialQuaternion;

    % 当太阳方向和磁场方向同时有效时，使用 TRIAD 得到直接定姿结果。
    % 这样比直接使用固定初值更容易收敛，也更符合实际星敏/磁强计组合使用方式。
    if measurements.sun.valid && measurements.mag.valid
        q0 = solve_attitude_from_sun_mag( ...
            measurements.sun.vector_b, ...
            measurements.mag.vector_b, ...
            measurements.reference.sun_i, ...
            measurements.reference.mag_i);
    end

    state = attitude_ekf_init(cfg, q0);
end

% 第一步: 根据角速度对姿态进行时间传播。
state = local_predict(state, omega_b, dt, cfg);

% 第二步: 用当前时刻的方向量测对姿态进行校正。
state = local_update(state, measurements, cfg);
end

function state = local_predict(state, omega_b, dt, cfg)
%LOCAL_PREDICT 根据姿态运动学方程进行预测。
%
% 输入:
%   state   : 上一时刻状态
%   omega_b : 当前角速度
%   dt      : 步长
%   cfg     : EKF 配置
%
% 输出:
%   state   : 写回预测后的 q_bi 和 P
%
% 主要步骤:
%   1. 根据四元数运动学模型 q_dot = f(q, omega) 做离散传播。
%   2. 对过程模型在当前点附近做线性化，得到状态转移雅可比 F。
%   3. 用标准协方差传播公式更新 P:
%        P(k|k-1) = F * P(k-1|k-1) * F' + Q
%
% 说明:
%   这里的 processFcn 是一个函数句柄，表示“给定当前四元数 q，经过 dt 秒后
%   的预测四元数”。后续数值雅可比会围绕它对状态进行微扰求导。

qPrev = state.q_bi;

% 构造过程模型句柄，便于后续重复调用和求数值雅可比。
processFcn = @(q) local_process_model(q, omega_b, dt);

% 利用当前姿态进行一步前向传播。
qPred = processFcn(qPrev);

% 对过程模型做数值线性化，得到状态转移矩阵 F。
F = local_numeric_jacobian(processFcn, qPrev, cfg.finiteDifferenceStep);

state.q_bi = qPred;
state.P = F * state.P * F' + cfg.processNoise * max(dt, eps);

% 协方差在数值运算后可能出现微小非对称，这里强制做一次对称化。
state.P = 0.5 * (state.P + state.P');
end

function state = local_update(state, measurements, cfg)
%LOCAL_UPDATE 利用太阳敏感器和磁力计量测更新姿态。
%
% 输入:
%   state        : 预测后的滤波器状态
%   measurements : 当前量测
%   cfg          : EKF 配置
%
% 输出:
%   state        : 量测更新后的状态
%
% 核心步骤:
%   1. 组装总量测向量 z、预测量测 h、量测雅可比 H 和量测噪声 R。
%   2. 计算量测残差 innovation = z - h。
%   3. 计算创新协方差 S 和卡尔曼增益 K。
%   4. 用 K 修正四元数和协方差。
%
% 注意:
%   这里直接对四元数进行加性修正，然后再归一化。
%   这是一种简化实现方式，工程验证阶段通常足够清晰易用。

[z, h, H, R] = local_measurement_system(state.q_bi, measurements, cfg);

% 若当前没有任何有效量测，则跳过更新，仅保留预测结果。
if isempty(z)
    state.lastInnovation = [];
    return;
end

% 量测残差，也称创新量，用于描述“测到的方向”和“按当前姿态预测的方向”之间的偏差。
innovation = z - h;

% 创新协方差 S 反映了残差的不确定性。
S = H * state.P * H' + R;

% 卡尔曼增益 K 决定“更相信预测”还是“更相信量测”。
K = state.P * H' / S;

% 用残差修正姿态，并重新归一化保证四元数模长为 1。
qUpdated = normalize_q(state.q_bi + K * innovation);

% 使用 Joseph 形式更新协方差，相比简单形式更稳健。
I = eye(size(state.P));
PUpdated = (I - K * H) * state.P * (I - K * H)' + K * R * K';

state.q_bi = qUpdated;
state.P = 0.5 * (PUpdated + PUpdated');
state.lastInnovation = innovation;
end

function qNext = local_process_model(q, omega_b, dt)
%LOCAL_PROCESS_MODEL 四元数姿态传播模型。
%
% 输入:
%   q       : 当前时刻姿态四元数
%   omega_b : 机体系角速度
%   dt      : 离散步长
%
% 输出:
%   qNext   : 传播后的下一时刻姿态四元数
%
% 传播方式:
%   采用最基础的一阶欧拉离散化:
%       q(k+1) = q(k) + dt * q_dot(k)
%   其中 q_dot 由 calc_qdot 计算。
%
% 说明:
%   这种写法简单直接，适合当前工程原型。若后续角速度更高或步长更大，
%   可替换为更高阶积分方法以减少离散误差。

qNext = normalize_q(q + dt * calc_qdot(q, omega_b));
end

function [z, h, H, R] = local_measurement_system(q_bi, measurements, cfg)
%LOCAL_MEASUREMENT_SYSTEM 构造 EKF 量测模型。
%
% 输入:
%   q_bi         : 当前用于预测量测的姿态四元数
%   measurements : 传感器量测结构体
%   cfg          : EKF 配置
%
% 输出:
%   z : 实际量测向量，按块堆叠
%   h : 预测量测向量，按块堆叠
%   H : 量测模型对状态的雅可比矩阵
%   R : 量测噪声协方差矩阵
%
% 量测模型思想:
%   对于方向类传感器，若已知某个参考方向在惯性系中的表达 v_i，
%   则在当前姿态 q_bi 下，其在机体系中的理论方向应为:
%       v_b_pred = C_bi(q) * v_i
%   将它与实际测得的 v_b_meas 比较，即可形成 EKF 的量测残差。
%
% 当前支持的量测源:
%   1. 太阳敏感器
%   2. 磁力计
%
% 实现细节:
%   1. 每类量测都先判断 valid 标志，只有有效量测才参与更新。
%   2. 每类量测都做单位化，以突出方向信息而非幅值信息。
%   3. 多种量测通过纵向拼接组成联合量测系统。

z = [];
h = [];
H = [];
R = [];

if measurements.sun.valid
    sunRef_i = local_unit_vector(measurements.reference.sun_i);
    zSun = local_unit_vector(measurements.sun.vector_b);

    % 太阳方向预测函数: 给定四元数 q，预测机体系下的太阳方向。
    hSunFcn = @(q) local_unit_vector(q2dcm(q) * sunRef_i);
    hSun = hSunFcn(q_bi);
    HSun = local_numeric_jacobian(hSunFcn, q_bi, cfg.finiteDifferenceStep);

    z = [z; zSun];
    h = [h; hSun];
    H = [H; HSun];
    R = blkdiag(R, cfg.sunMeasurementNoise);
end

if measurements.mag.valid
    magRef_i = local_unit_vector(measurements.reference.mag_i);
    zMag = local_unit_vector(measurements.mag.vector_b);

    % 磁场方向预测函数: 给定四元数 q，预测机体系下的磁场方向。
    hMagFcn = @(q) local_unit_vector(q2dcm(q) * magRef_i);
    hMag = hMagFcn(q_bi);
    HMag = local_numeric_jacobian(hMagFcn, q_bi, cfg.finiteDifferenceStep);

    z = [z; zMag];
    h = [h; hMag];
    H = [H; HMag];
    R = blkdiag(R, cfg.magMeasurementNoise);
end
end

function J = local_numeric_jacobian(funHandle, x0, step)
%LOCAL_NUMERIC_JACOBIAN 使用中心差分近似计算雅可比矩阵。
%
% 输入:
%   funHandle : 待求导函数句柄，输入为状态向量，输出为函数值
%   x0        : 当前线性化点
%   step      : 差分步长
%
% 输出:
%   J         : 数值雅可比矩阵
%
% 计算方法:
%   对第 i 个状态分量施加正负微扰，使用中心差分公式:
%       J(:,i) = (f(x0 + dx_i) - f(x0 - dx_i)) / (2 * step)
%
% 说明:
%   1. 中心差分比前向差分精度更高。
%   2. 对四元数做微扰后要重新归一化，否则会偏离单位四元数流形。
%   3. step 太小会放大舍入误差，太大又会降低线性化精度，因此需要折中。

y0 = funHandle(x0);
J = zeros(numel(y0), numel(x0));

for idx = 1:numel(x0)
    perturbation = zeros(size(x0));
    perturbation(idx) = step;

    yPlus = funHandle(normalize_q(x0 + perturbation));
    yMinus = funHandle(normalize_q(x0 - perturbation));
    J(:, idx) = (yPlus - yMinus) / (2.0 * step);
end
end