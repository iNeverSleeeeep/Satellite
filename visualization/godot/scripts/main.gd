extends Node3D

const UDP_PORT := 4242
const DISTANCE_SCALE := 1.0 / 1000000.0
const EARTH_RADIUS_M := 6378137.0
const ATMOSPHERE_SCALE := 1.025
const CAMERA_LERP := 3.5

@onready var world_environment: WorldEnvironment = $WorldEnvironment
@onready var sun_light: DirectionalLight3D = $SunLight
@onready var camera: Camera3D = $Camera3D
@onready var status_label: Label = $Ui/StatusLabel

var udp := PacketPeerUDP.new()
var latest_packet: Dictionary = {}
var has_packet := false
var earth_mesh_instance: MeshInstance3D
var atmosphere_mesh_instance: MeshInstance3D
var satellite_root: Node3D
var satellite_body: MeshInstance3D
var solar_panel_left: MeshInstance3D
var solar_panel_right: MeshInstance3D
var camera_velocity := Vector3.ZERO

func _ready() -> void:
	_setup_environment()
	_build_earth()
	_build_satellite()
	_start_receiver()

func _process(delta: float) -> void:
	_poll_packets()
	if has_packet:
		_update_from_packet(delta)
	else:
		earth_mesh_instance.rotate_y(0.05 * delta)
		_update_status_label(false, 0.0, 0.0, 0.0)

func _setup_environment() -> void:
	var environment := Environment.new()
	environment.background_mode = Environment.BG_COLOR
	environment.background_color = Color(0.01, 0.02, 0.05)
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	environment.ambient_light_color = Color(0.18, 0.22, 0.30)
	environment.ambient_light_energy = 1.0
	environment.glow_enabled = true
	environment.glow_intensity = 0.08
	world_environment.environment = environment
	sun_light.light_color = Color(1.0, 0.97, 0.92)

func _build_earth() -> void:
	earth_mesh_instance = MeshInstance3D.new()
	var earth_mesh := SphereMesh.new()
	earth_mesh.radius = EARTH_RADIUS_M * DISTANCE_SCALE
	earth_mesh.height = earth_mesh.radius * 2.0
	earth_mesh.radial_segments = 64
	earth_mesh.rings = 32
	earth_mesh_instance.mesh = earth_mesh

	var earth_material := StandardMaterial3D.new()
	earth_material.albedo_color = Color(0.10, 0.34, 0.72)
	earth_material.roughness = 0.78
	earth_material.metallic = 0.02
	earth_material.emission_enabled = true
	earth_material.emission = Color(0.01, 0.04, 0.08)
	earth_mesh_instance.material_override = earth_material
	add_child(earth_mesh_instance)

	atmosphere_mesh_instance = MeshInstance3D.new()
	var atmosphere_mesh := SphereMesh.new()
	atmosphere_mesh.radius = EARTH_RADIUS_M * DISTANCE_SCALE * ATMOSPHERE_SCALE
	atmosphere_mesh.height = atmosphere_mesh.radius * 2.0
	atmosphere_mesh.radial_segments = 48
	atmosphere_mesh.rings = 24
	atmosphere_mesh_instance.mesh = atmosphere_mesh

	var atmosphere_material := StandardMaterial3D.new()
	atmosphere_material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	atmosphere_material.albedo_color = Color(0.32, 0.64, 1.0, 0.12)
	atmosphere_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	atmosphere_material.cull_mode = BaseMaterial3D.CULL_DISABLED
	atmosphere_material.emission_enabled = true
	atmosphere_material.emission = Color(0.10, 0.24, 0.48)
	atmosphere_material.emission_energy_multiplier = 0.35
	atmosphere_mesh_instance.material_override = atmosphere_material
	add_child(atmosphere_mesh_instance)

func _build_satellite() -> void:
	satellite_root = Node3D.new()
	add_child(satellite_root)

	satellite_body = MeshInstance3D.new()
	var body_mesh := BoxMesh.new()
	body_mesh.size = Vector3(0.18, 0.12, 0.12)
	satellite_body.mesh = body_mesh

	var body_material := StandardMaterial3D.new()
	body_material.albedo_color = Color(0.85, 0.87, 0.92)
	body_material.roughness = 0.35
	satellite_body.material_override = body_material
	satellite_root.add_child(satellite_body)

	solar_panel_left = MeshInstance3D.new()
	var panel_mesh := BoxMesh.new()
	panel_mesh.size = Vector3(0.02, 0.32, 0.14)
	solar_panel_left.mesh = panel_mesh
	solar_panel_left.position = Vector3(-0.16, 0.0, 0.0)

	var panel_material := StandardMaterial3D.new()
	panel_material.albedo_color = Color(0.08, 0.14, 0.24)
	panel_material.metallic = 0.1
	panel_material.roughness = 0.2
	solar_panel_left.material_override = panel_material
	satellite_root.add_child(solar_panel_left)

	solar_panel_right = MeshInstance3D.new()
	solar_panel_right.mesh = panel_mesh
	solar_panel_right.position = Vector3(0.16, 0.0, 0.0)
	solar_panel_right.material_override = panel_material
	satellite_root.add_child(solar_panel_right)

func _start_receiver() -> void:
	var err := udp.bind(UDP_PORT, "127.0.0.1")
	if err != OK:
		push_error("Failed to bind UDP receiver on port %d" % UDP_PORT)
		status_label.text = "UDP bind failed on port %d" % UDP_PORT
	else:
		status_label.text = "Waiting for MATLAB telemetry on UDP %d" % UDP_PORT

func _poll_packets() -> void:
	while udp.get_available_packet_count() > 0:
		var payload := udp.get_packet().get_string_from_utf8()
		var parsed = JSON.parse_string(payload)
		if parsed is Dictionary:
			latest_packet = parsed
			has_packet = true

func _update_from_packet(delta: float) -> void:
	var position_m := _array_to_vector3(latest_packet.get("position_i_m", []))
	var velocity_mps := _array_to_vector3(latest_packet.get("velocity_i_mps", []))
	var q_bi_array: Array = latest_packet.get("q_bi", [])
	var sim_time_s := float(latest_packet.get("sim_time_s", 0.0))

	satellite_root.position = position_m * DISTANCE_SCALE
	if q_bi_array.size() == 4:
		satellite_root.basis = Basis(_godot_body_orientation_from_q_bi(q_bi_array))

	earth_mesh_instance.rotate_y(0.03 * delta)
	atmosphere_mesh_instance.rotate_y(0.04 * delta)
	_update_camera(delta)

	var altitude_km := max(position_m.length() - EARTH_RADIUS_M, 0.0) / 1000.0
	var speed_kmps := velocity_mps.length() / 1000.0
	_update_status_label(true, sim_time_s, altitude_km, speed_kmps)

func _update_camera(delta: float) -> void:
	var target := satellite_root.position * 0.65
	var desired_position := target + Vector3(0.0, 5.0, 12.0)
	camera.global_position = camera.global_position.lerp(desired_position, clamp(delta * CAMERA_LERP, 0.0, 1.0))
	camera.look_at(target, Vector3.UP)

func _update_status_label(is_connected: bool, sim_time_s: float, altitude_km: float, speed_kmps: float) -> void:
	if is_connected:
		status_label.text = "MATLAB -> Godot live\nUDP %d connected\nSim Time: %.2f s\nAltitude: %.1f km\nSpeed: %.3f km/s" % [UDP_PORT, sim_time_s, altitude_km, speed_kmps]
	else:
		status_label.text = "Waiting for MATLAB telemetry\nUDP %d\nRun run_godot_stream_demo in MATLAB" % UDP_PORT

func _array_to_vector3(values: Array) -> Vector3:
	if values.size() < 3:
		return Vector3.ZERO
	return Vector3(float(values[0]), float(values[2]), -float(values[1]))

func _godot_body_orientation_from_q_bi(q_bi: Array) -> Quaternion:
	var body_from_inertial := Quaternion(float(q_bi[1]), float(q_bi[3]), -float(q_bi[2]), float(q_bi[0])).normalized()
	return body_from_inertial.inverse()
