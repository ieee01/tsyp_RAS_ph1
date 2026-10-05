# The command post decides WHO goes and WHAT the objective is - never HOW.
uint16 mission_id
# Robot this brief is routed to by the Outside Network (e.g. "executor").
string robot_id
# Beacon whose memory is the objective, and its event type (filled by the gateway).
uint16 target_beacon_id
# All objectives in the order the command post wants them handled (by priority);
# target_beacon_id is the first of them.
uint16[] target_beacon_ids
string target_event
uint8 target_type
# Deprecated: hazard policy belongs to the robot. Empty means "robot decides".
uint8[] avoid_event_types
string objective
builtin_interfaces/Time issued_at
