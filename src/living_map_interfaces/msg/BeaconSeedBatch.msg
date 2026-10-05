# Everything the outside world knows, handed to a robot before it enters:
# every received beacon frame plus the last map the Writer uploaded.
uint16 mission_id
builtin_interfaces/Time issued_at
uint16 frame_size
uint16 frame_count
uint8[] data
# Last Writer SLAM map received over the radio mesh (empty if none arrived).
nav_msgs/OccupancyGrid map
