function godot_stream_close(stream)
%GODOT_STREAM_CLOSE Close a UDP stream created for Godot visualization.

if nargin < 1 || ~isstruct(stream) || ~isfield(stream, 'udp')
    return;
end

if isa(stream.udp, 'udpport')
    clear stream;
end
end
