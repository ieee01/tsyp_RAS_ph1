#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/_env.sh"
if [[ ! -f install/_local_setup_util_sh.py ]]; then
  echo "[AutoDemo] Workspace install is missing/incomplete; rebuilding it now."
  ./scripts/build.sh
  source "$(dirname "$0")/_env.sh"
fi
command -v ros2 >/dev/null || { echo "ROS 2 Jazzy required for the automated simulation demo."; exit 2; }
mkdir -p runtime

if ! ros2 node list 2>/dev/null | grep -qx '/living_map/gateway'; then
  echo "[AutoDemo] Launching deterministic recovery mode in the background."
  echo "[AutoDemo] NOTE: this uses the explicitly labelled waypoint fallback; normal run_demo.sh uses frontier exploration."
  LIVING_MAP_AUTO_EXPLORE=true ./scripts/run_demo.sh --fallback > runtime/auto_demo_stack.log 2>&1 &
  echo $! > runtime/auto_demo.pid
fi

# Wait on state, not an assumed launch duration.
ready=0
for _ in {1..120}; do
  if curl -fsS http://localhost:8081/api/state >/dev/null 2>&1; then ready=1; break; fi
  sleep 1
done
[[ "$ready" -eq 1 ]] || {
  echo "[AutoDemo] Command Post did not become ready through the link. See runtime/auto_demo_stack.log"; exit 1;
}

health_ready=0
for _ in {1..60}; do
  if ./scripts/health_check.sh > runtime/health_check.log 2>&1; then health_ready=1; break; fi
  sleep 1
done
[[ "$health_ready" -eq 1 ]] || {
  echo "[AutoDemo] ROS/Nav2 health checks did not pass. See runtime/health_check.log"; exit 1;
}

event_ready=0
count=0
for _ in {1..300}; do
  result=$(python - <<'PY2' 2>/dev/null || true
import json, os
p = "runtime/beacons.json"
if not os.path.exists(p):
    print("0 0")
else:
    values = list(json.load(open(p)).values())
    types = {int(v.get("event_type", 0)) for v in values}
    print(len(values), 1 if {1, 2, 4} <= types else 0)
PY2
)
  read -r count event_ready <<<"${result:-0 0}"
  [[ "$event_ready" -eq 1 ]] && break
  sleep 1
done
[[ "$event_ready" -eq 1 ]] || { echo "[AutoDemo] Timed out waiting for VICTIM, GAS_HAZARD and FIRE memories"; exit 1; }

# Persistence is immediate, while the lossy RF model may need another
# rebroadcast before the command post has both semantic memories to seed.
gateway_events_ready=0
for _ in {1..60}; do
  if curl -fsS http://localhost:8081/api/state 2>/dev/null | python -c 'import json,sys; types={int(m.get("event_type",0)) for m in json.load(sys.stdin).get("memories",[])}; raise SystemExit(0 if {1,2,4} <= types else 1)' 2>/dev/null; then
    gateway_events_ready=1
    break
  fi
  sleep 1
done
[[ "$gateway_events_ready" -eq 1 ]] || { echo "[AutoDemo] Command post did not receive all three semantic memories over RF"; exit 1; }

echo "[AutoDemo] VICTIM, GAS_HAZARD and FIRE memories are persistent ($count total); injecting Writer failure."
./scripts/fail_writer.sh

failed_seen=0
for _ in {1..30}; do
  if curl -fsS http://localhost:8081/api/state 2>/dev/null | python -c 'import json,sys; s=json.load(sys.stdin); raise SystemExit(0 if s.get("robots",{}).get("writer",{}).get("state")=="FAILED" else 1)' 2>/dev/null; then
    failed_seen=1
    break
  fi
  sleep 1
done
[[ "$failed_seen" -eq 1 ]] || { echo "[AutoDemo] Writer FAILED state did not reach the gateway"; exit 1; }

echo "[AutoDemo] Writer failure confirmed while persistent memories remain."
echo "[AutoDemo] Command post briefs the rescue robots -> long-distance link -> Gateway."
./scripts/start_executor.sh

mission_result=""
for _ in {1..300}; do
  # The command post completes only when every dispatched robot is done.
  mission_result=$(curl -fsS http://localhost:8081/api/state 2>/dev/null | python -c 'import json,sys; p=json.load(sys.stdin).get("demo_control",{}).get("phase",""); print({"COMPLETE":"MISSION COMPLETE","ERROR":"MISSION FAILED"}.get(p,""))' 2>/dev/null || true)
  if [[ "$mission_result" == "MISSION COMPLETE" || "$mission_result" == "MISSION FAILED" ]]; then
    break
  fi
  sleep 1
done

if [[ "$mission_result" == "MISSION COMPLETE" ]]; then
  echo "[AutoDemo] MISSION COMPLETE"
  echo "[AutoDemo] Evidence remains visible at http://localhost:8081 and in RViz."
  exit 0
fi

echo "[AutoDemo] Mission did not complete successfully (status: ${mission_result:-unknown})."
echo "[AutoDemo] See runtime/auto_demo_stack.log"
exit 1
