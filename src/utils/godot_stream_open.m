function stream = godot_stream_open(remoteHost, remotePort)
%GODOT_STREAM_OPEN Open a UDP stream for Godot live visualization.

arguments
    remoteHost (1, :) char = '127.0.0.1'
    remotePort (1, 1) double = 4242
end

stream = struct();
stream.remoteHost = remoteHost;
stream.remotePort = remotePort;
stream.udp = udpport("datagram", "IPV4");
end
