function run_godot_stream_demo()
%RUN_GODOT_STREAM_DEMO Stream a simple real-time orbit demo to Godot.

setup_paths();
env = environment_config();
simConfig = default_sim_config();

remoteHost = '127.0.0.1';
remotePort = 4242;
stream = godot_stream_open(remoteHost, remotePort);
cleanupObj = onCleanup(@() godot_stream_close(stream)); %#ok<NASGU>

radius = norm(simConfig.X_i0);
meanMotion = sqrt(env.earthMuM3S2 / radius^3);
updateRateHz = 30.0;
dtWall = 1.0 / updateRateHz;
spinRateRadS = deg2rad(12.0);

fprintf('Streaming to Godot at udp://%s:%d\n', remoteHost, remotePort);
fprintf('Open the Godot project and run the main scene to view telemetry.\n');

startWall = tic;
nextTick = 0.0;
while true
    t = toc(startWall);
    phase = meanMotion * t;

    X_i = radius * [cos(phase); sin(phase); 0.12 * sin(0.25 * phase)];
    V_i = radius * meanMotion * [-sin(phase); cos(phase); 0.03 * cos(0.25 * phase)];

    yaw = spinRateRadS * t;
    q_bi = [cos(0.5 * yaw); 0.0; 0.0; sin(0.5 * yaw)];
    omega_b = [0.0; 0.0; spinRateRadS];

    godot_stream_send_state(stream, t, X_i, V_i, q_bi, omega_b);

    nextTick = nextTick + dtWall;
    sleepTime = nextTick - toc(startWall);
    if sleepTime > 0.0
        pause(sleepTime);
    else
        nextTick = toc(startWall);
    end
end
end
