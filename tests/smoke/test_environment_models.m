function test_environment_models()
%TEST_ENVIRONMENT_MODELS 环境与扰动模型的基础冒烟测试。

setup_paths();

env = environment_config();
spacecraft = spacecraft_params();
X_i = [7000e3; 0; 0];
V_i = [0; 7.5e3; 0];
DCM_bi = eye(3);
t = 120.0;
sunVector_i = calc_sun_vector_i(t, env);
B_i = calc_magnetic_field_i(X_i, env);

drag_b = calc_atmospheric_drag_b(X_i, V_i, DCM_bi, spacecraft, env);
srp_b = calc_solar_radiation_pressure_b(sunVector_i, DCM_bi, spacecraft, env);
gg_b = calc_gravity_gradient_torque_b(X_i, DCM_bi, spacecraft.inertiaKgM2, env);
mag_b = calc_magnetic_torque_b(B_i, DCM_bi, spacecraft.residualDipoleAm2);
srp_torque_b = calc_solar_pressure_torque_b(sunVector_i, DCM_bi, spacecraft, env);
drag_torque_b = calc_aerodynamic_torque_b(X_i, V_i, DCM_bi, spacecraft, env);

assert(all(size(sunVector_i) == [3, 1]));
assert(all(size(B_i) == [3, 1]));
assert(all(size(drag_b) == [3, 1]));
assert(all(size(srp_b) == [3, 1]));
assert(all(size(gg_b) == [3, 1]));
assert(all(size(mag_b) == [3, 1]));
assert(all(size(srp_torque_b) == [3, 1]));
assert(all(size(drag_torque_b) == [3, 1]));
assert(all(isfinite(sunVector_i)));
assert(all(isfinite(B_i)));
assert(all(isfinite(drag_b)));
assert(all(isfinite(srp_b)));
assert(all(isfinite(gg_b)));
assert(all(isfinite(mag_b)));
assert(all(isfinite(srp_torque_b)));
assert(all(isfinite(drag_torque_b)));
end
