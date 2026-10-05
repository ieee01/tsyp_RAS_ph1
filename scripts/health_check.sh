#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/_env.sh"
command -v ros2 >/dev/null || { echo "ROS 2 is not available on this host."; exit 2; }

echo "================================"
echo "LivingMap System Health"
echo "================================"
missing=0
# Use one settled discovery snapshot instead of ten transient daemon reads.
livingmap_nodes="$(timeout 6 ros2 node list --no-daemon --spin-time 2 2>/dev/null || true)"

check_node() {
  local label=$1 node=$2
  if grep -qx "$node" <<<"$livingmap_nodes"; then
    printf '%-22s ONLINE\n' "$label"
  else
    printf '%-22s MISSING\n' "$label"
    missing=$((missing+1))
  fi
}

check_lifecycle_active() {
  local label=$1 node=$2
  local state
  for _ in {1..3}; do
    state="$(timeout 5 ros2 lifecycle get "$node" 2>/dev/null || true)"
    if grep -qi "^active" <<<"$state"; then break; fi
  done
  if grep -qi "^active" <<<"$state"; then
    printf '%-22s ACTIVE\n' "$label"
  else
    printf '%-22s NOT ACTIVE\n' "$label"
    missing=$((missing+1))
  fi
}

check_nav_plugin() {
  local label=$1 node=$2
  local plugins plugin
  plugins="$(timeout 5 ros2 param get "$node" controller_plugins 2>/dev/null || true)"
  plugin="$(timeout 5 ros2 param get "$node" FollowPath.plugin 2>/dev/null || true)"
  if grep -q "FollowPath" <<<"$plugins" && grep -q "RegulatedPurePursuitController" <<<"$plugin"; then
    printf '%-22s RPP/FollowPath\n' "$label"
  else
    printf '%-22s WRONG/MISSING\n' "$label"
    printf '  controller_plugins: %s\n' "$plugins"
    printf '  FollowPath.plugin:  %s\n' "$plugin"
    missing=$((missing+1))
  fi
}

check_node Writer /living_map/writer
check_node Executor /living_map/executor
check_node Gateway /living_map/gateway
check_node Link_Simulator /living_map/long_distance_link
check_node Command_Post /living_map/command_post
check_node RF_Network /living_map/radio
check_node Beacons /living_map/beacons
check_node SLAM /slam_toolbox
check_node Writer_Nav2 /writer/controller_server
check_node Executor_Nav2 /executor/controller_server

check_lifecycle_active Writer_Controller /writer/controller_server
check_lifecycle_active Executor_Controller /executor/controller_server
check_lifecycle_active Writer_Planner /writer/planner_server
check_lifecycle_active Executor_Planner /executor/planner_server
check_nav_plugin Writer_FollowPath /writer/controller_server
check_nav_plugin Executor_FollowPath /executor/controller_server

if curl -fsS http://localhost:8081/api/state >/dev/null 2>&1; then
  printf '%-22s ONLINE\n' Dashboard
else
  printf '%-22s MISSING\n' Dashboard
  missing=$((missing+1))
fi

echo
echo "Required package discovery:"
for pkg in living_map_bringup living_map_sim living_map_gateway living_map_link_sim living_map_command_post living_map_executor; do
  if timeout 5 ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    printf '  %-20s OK\n' "$pkg"
  else
    printf '  %-20s NOT FOUND\n' "$pkg"
    missing=$((missing+1))
  fi
done

echo
echo "Important LivingMap services:"
timeout 5 ros2 service list 2>/dev/null | grep living_map || true

if [[ "$missing" -eq 0 ]]; then
  echo
  echo "SYSTEM READY"
else
  echo
  echo "SYSTEM NOT READY ($missing required check(s) failed)"
  exit 1
fi
