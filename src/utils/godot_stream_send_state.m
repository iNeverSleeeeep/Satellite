function godot_stream_send_state(stream, t, X_i, V_i, q_bi, omega_b)
%GODOT_STREAM_SEND_STATE Send one spacecraft state packet to Godot.

arguments
    stream (1, 1) struct
    t (1, 1) double
    X_i (3, 1) double
    V_i (3, 1) double
    q_bi (4, 1) double
    omega_b (3, 1) double
end

packet = struct();
packet.version = 1;
packet.sim_time_s = t;
packet.position_i_m = reshape(X_i, 1, []);
packet.velocity_i_mps = reshape(V_i, 1, []);
packet.q_bi = reshape(q_bi ./ max(norm(q_bi), eps), 1, []);
packet.omega_b_radps = reshape(omega_b, 1, []);

payload = jsonencode(packet);
write(stream.udp, unicode2native(payload, 'UTF-8'), "uint8", stream.remoteHost, stream.remotePort);
end
