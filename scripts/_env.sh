#!/usr/bin/env bash
# Shared environment setup. Safe to source from scripts that enable `set -u`.

_livingmap_restore_nounset=0
case $- in
  *u*) _livingmap_restore_nounset=1; set +u ;;
esac

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "$ROOT"

# ROS / ament setup scripts may legitimately inspect unset variables. Keep nounset
# disabled while sourcing them, then restore the caller's shell mode.
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi
if [[ -f .venv/bin/activate ]]; then
  source .venv/bin/activate
  # ROS Python entry points retain /usr/bin/python3 in their shebang, so PATH
  # activation alone does not expose packages installed in this workspace venv.
  _livingmap_venv_site="$ROOT/.venv/lib/python3.12/site-packages"
  if [[ -d "$_livingmap_venv_site" ]]; then
    export PYTHONPATH="$_livingmap_venv_site:${PYTHONPATH:-}"
  fi
fi
if [[ "${LIVING_MAP_SKIP_WORKSPACE_SETUP:-0}" != "1" && -f install/setup.bash ]]; then
  if [[ -f install/_local_setup_util_sh.py ]]; then
    source install/setup.bash
  else
    echo "[LivingMap] Incomplete install detected; run ./scripts/build.sh" >&2
  fi
fi

export LIVING_MAP_ROOT="$ROOT"
export LIVING_MAP_RUNTIME_DIR="${LIVING_MAP_RUNTIME_DIR:-$ROOT/runtime}"
# VMware shared-memory locks can outlive a crashed participant. UDPv4 keeps
# command-line health probes and launched nodes in the same reliable transport.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
# ROS 2 discovery uses the Jazzy default (SUBNET). LOCALHOST was tried: without
# multicast on the loopback interface it only reaches the first few of the ~40
# demo processes, so most nodes never discover each other. For recordings, keep
# the network stable (or disconnect it) so discovery traffic is not disturbed.
if [[ -n "${LIVING_MAP_DISCOVERY_RANGE:-}" ]]; then
  export ROS_AUTOMATIC_DISCOVERY_RANGE="$LIVING_MAP_DISCOVERY_RANGE"
fi
export PYTHONPATH="$ROOT/src/living_map_beacons:$ROOT/src/living_map_radio:$ROOT/src/living_map_gateway:$ROOT/src/living_map_executor:$ROOT/src/living_map_link_sim:$ROOT/src/living_map_command_post:${PYTHONPATH:-}"

if [[ "$_livingmap_restore_nounset" -eq 1 ]]; then
  set -u
fi
unset _livingmap_restore_nounset _livingmap_venv_site
