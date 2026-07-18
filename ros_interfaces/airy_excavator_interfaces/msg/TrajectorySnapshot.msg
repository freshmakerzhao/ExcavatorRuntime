std_msgs/Header header
string trajectory_id
string trajectory_sha256
string mission_id
string mission_sha256
string mission_phase
string task_mode
string planning_scope
string control_stage
string workspace_constraint
bool execution_eligible
builtin_interfaces/Time source_bucket_tip_stamp
builtin_interfaces/Time source_local_map_stamp
# Time at which the live inputs above were validated and frozen for planning.
builtin_interfaces/Time inputs_frozen_at
# Latest time at which a consumer may accept this snapshot; tracking_timeout_s governs an accepted run.
builtin_interfaces/Time valid_until
string input_source
string map_source
string clock_mode
geometry_msgs/Point[] waypoints
float64 waypoint_tolerance_m
float64 waypoint_dwell_s
float64 tracking_timeout_s
