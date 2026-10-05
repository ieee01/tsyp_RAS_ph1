uint8 protocol_version
uint16 mission_id
uint16 beacon_id
uint16 sequence
builtin_interfaces/Time written_at
float32 x
float32 y
# Guidance vector: bearing (map frame, rad) and range to the event (standoff
# beacons) or to the next beacon of the chain.
float32 yaw
float32 range_m
# Role bit field: 1 JUNCTION, 2 RELAY, 4 STANDOFF, 8 ENTRANCE, 16 BREADCRUMB.
uint8 flags
uint8 event_type
uint8 severity
float32 confidence
uint16 previous_beacon_id
uint16 suggested_next_beacon_id
uint32 ttl_seconds
uint32 crc32
float32 age_seconds
string freshness
string event_id
