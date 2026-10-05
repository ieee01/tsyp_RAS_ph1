#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/_env.sh"
if [[ ! -f install/_local_setup_util_sh.py ]]; then
  echo "[LivingMap] Workspace install is missing/incomplete; rebuilding it now."
  ./scripts/build.sh
  source "$(dirname "$0")/_env.sh"
fi
command -v ros2 >/dev/null || { echo "ROS 2 Jazzy required. For core validation here run: ./tools/headless_demo.py"; exit 2; }
mkdir -p runtime
# VMware shared-memory locks can survive an interrupted Fast DDS process and
# prevent reliable discovery on the next judge run. The demo is entirely local,
# so UDP loopback is deterministic and avoids that host-specific failure mode.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
if pgrep -f '^gz sim server$' >/dev/null 2>&1; then
  echo "A stale Gazebo server is running. Run ./scripts/reset_demo.sh before starting a new demo."
  exit 2
fi
cleanup_demo() {
  pkill -f '^gz sim server$' 2>/dev/null || true
  pkill -f "$LIVING_MAP_ROOT/install/" 2>/dev/null || true
}
trap cleanup_demo EXIT INT TERM
fallback=false
speed="${LIVING_MAP_SPEED:-normal}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --fallback) fallback=true ;;
    --speed) [[ $# -ge 2 ]] || { echo "--speed needs a value: normal, fast or very_fast"; exit 2; }; speed="$2"; shift ;;
    --speed=*) speed="${1#--speed=}" ;;
    *) echo "Usage: $0 [--fallback] [--speed normal|fast|very_fast]"; exit 2 ;;
  esac
  shift
done
speed="${speed//-/_}"
case "$speed" in
  normal|fast|very_fast) ;;
  *) echo "Unknown speed '$speed'; choose normal, fast or very_fast"; exit 2 ;;
esac
export LIVING_MAP_SPEED="$speed"
echo "[LivingMap] Robot speed preset: $speed"
if [[ "$fallback" == true ]]; then
  export LIVING_MAP_DEMO_MODE=guided
  echo "[LivingMap] DEMO FALLBACK requested: deterministic Writer waypoints (frontier exploration disabled)."
else
  export LIVING_MAP_DEMO_MODE=autonomous
fi
ros2 launch living_map_bringup demo.launch.py demo_fallback:="$fallback" speed:="$speed" auto_explore:="${LIVING_MAP_AUTO_EXPLORE:-false}" rviz:="${LIVING_MAP_RVIZ:-true}"
