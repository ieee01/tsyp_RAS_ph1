#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# Stop every demo launch together with its whole process tree. Some Nav2 nodes
# (e.g. lifecycle managers) only reference temporary parameter files, so a
# path-based match alone would leave them running and interfering with the
# next bringup.
descendants() {
  local child
  for child in $(pgrep -P "$1" 2>/dev/null); do
    echo "$child"
    descendants "$child"
  done
}
# Never stop this script or the helper that started it: the dashboard's Restart
# button runs this reset from a helper that is a child of the command post,
# inside the very process tree being stopped. The helper runs in its own session,
# so protect only the callers in this script's session. Their ancestors (the
# command post and its launch) must still stop: a launch left alive respawns
# Gazebo with the old world next to the new one.
protected=" "
own_session=$(ps -o sid= -p $$ 2>/dev/null | tr -d ' ')
pid=$$
while [[ -n "$pid" && "$pid" -gt 1 ]]; do
  [[ "$(ps -o sid= -p "$pid" 2>/dev/null | tr -d ' ')" == "$own_session" ]] || break
  protected="$protected$pid "
  pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
done
# Match only the real processes (a launch, or bash running run_demo.sh), never a
# shell that merely mentions them.
tree_pids=""
for root in $(pgrep -f "^/usr/bin/python3 /opt/ros/[^ ]+/bin/ros2 launch living_map_bringup demo.launch.py" 2>/dev/null) \
            $(pgrep -f "^(/usr)?/?(bin/)?bash [^ ]*run_demo\.sh( |$)" 2>/dev/null); do
  for candidate in $root $(descendants "$root"); do
    case "$protected" in *" $candidate "*) continue ;; esac
    tree_pids="$tree_pids $candidate"
  done
done
if [[ -n "${tree_pids// /}" ]]; then
  kill -INT $tree_pids 2>/dev/null || true
  for _ in {1..50}; do
    alive=""
    for pid in $tree_pids; do if kill -0 "$pid" 2>/dev/null; then alive="$alive $pid"; fi; done
    if [[ -z "$alive" ]]; then break; fi
    sleep 0.2
  done
  if [[ -n "${alive:-}" ]]; then kill -TERM $alive 2>/dev/null || true; sleep 1; fi
  for pid in $tree_pids; do if kill -0 "$pid" 2>/dev/null; then kill -KILL "$pid" 2>/dev/null || true; fi; done
fi
# Anything still running in this demo's robot namespaces is a leftover.
pkill -KILL -f -- "^/opt/ros/[^ ]+/lib/.* __ns:=/(writer|executor|firebot)( |$)" 2>/dev/null || true
# An older abruptly killed launch can leave the bridge outside install/ alive.
# Scope cleanup to this demo's pair of Writer/Executor command topics.
pkill -f '^/opt/ros/.*/lib/ros_gz_bridge/parameter_bridge /clock@rosgraph_msgs/msg/Clock.* /writer/cmd_vel@.* /executor/cmd_vel@' 2>/dev/null || true
pkill -f "^gz sim -r .*mine\.sdf" 2>/dev/null || true
pkill -f '^gz sim server$' 2>/dev/null || true
pkill -f '^gz sim gui$' 2>/dev/null || true
# A Gazebo server that is still shutting down makes the next one hang at start-up
# (no clock, no robots). Wait for it to exit, then force it.
for _ in {1..75}; do
  if ! pgrep -f '^gz sim (server|gui)$' >/dev/null 2>&1; then break; fi
  sleep 0.2
done
pkill -KILL -f '^gz sim (server|gui)$' 2>/dev/null || true
pkill -f '^/opt/ros/.*/lib/tf2_ros/static_transform_publisher .* (writer/odom (executor|firebot)/odom|(writer|executor|firebot)/base_link (writer|executor|firebot)/laser) --ros-args' 2>/dev/null || true
# A killed launch parent can leave ROS children orphaned. Scope cleanup to
# executables and parameter files under this workspace so unrelated ROS work is
# not touched.
pkill -f "^(/usr/bin/python3 )?$ROOT/install/" 2>/dev/null || true
sleep 1
rm -f "$ROOT/runtime/beacons.json" "$ROOT/runtime/beacons.json.bak" "$ROOT/runtime/beacons.json.corrupt"* "$ROOT/runtime/dashboard_state.json"
echo "LivingMap demo processes stopped and persistent state cleared. Run ./scripts/run_demo.sh for a clean replay."
