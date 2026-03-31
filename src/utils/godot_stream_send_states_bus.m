function godot_stream_send_states_bus(stream, t, states)
%GODOT_STREAM_SEND_STATES_BUS Send one project state bus sample to Godot.

arguments
    stream (1, 1) struct
    t (1, 1) double
    states (1, 1) struct
end

godot_stream_send_state(stream, t, states.X_i, states.V_i, states.q_bi, states.omega_b);
end
