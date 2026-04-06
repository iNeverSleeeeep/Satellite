function value = actuator_sign_with_zero(vectorIn)
%ACTUATOR_SIGN_WITH_ZERO 对接近零的量保持零符号。
value = sign(vectorIn);
value(abs(vectorIn) < eps) = 0.0;
end