function state = attitude_ekf_init(cfg, q0)
%ATTITUDE_EKF_INIT Initialize the quaternion-based attitude EKF state.

if nargin < 2 || isempty(q0)
    q0 = cfg.initialQuaternion;
end

state.q_bi = normalize_q(q0);
state.P = cfg.initialCovariance;
state.isInitialized = true;
state.lastInnovation = [];
end
