#!/usr/bin/env python3
"""Local simulation replay harness; not a robot-to-command-post communication path."""
from pathlib import Path
import os
import shutil
import subprocess
import sys
import time

root = Path(__file__).resolve().parents[1]
mode = sys.argv[1] if len(sys.argv) >= 2 else "autonomous"
speed = sys.argv[2] if len(sys.argv) >= 3 else "normal"
if mode not in {"autonomous", "guided"}:
    raise SystemExit("Invalid navigation mode")
if speed not in {"normal", "fast", "very_fast"}:
    raise SystemExit("Invalid speed preset")
time.sleep(1)  # Allow HTTP acknowledgement before the command post is stopped.
archive = root / "runtime" / ("replay-" + time.strftime("%Y%m%d-%H%M%S"))
archive.mkdir(parents=True, exist_ok=True)
for pattern in ("beacons.json*", "dashboard_state.json", "demo.log"):
    for path in (root / "runtime").glob(pattern):
        if path.is_file():
            shutil.copy2(path, archive / path.name)
subprocess.run([str(root / "scripts/reset_demo.sh")], cwd=root, check=True)
# Gazebo shutdown can outlast reset_demo's one-second grace period.
for _ in range(75):
    if subprocess.run(["pgrep", "-f", "^gz sim server$"], stdout=subprocess.DEVNULL).returncode != 0:
        break
    time.sleep(0.2)
else:
    subprocess.run(["pkill", "-KILL", "-f", "^gz sim server$"], check=False)
    time.sleep(0.5)
env = dict(os.environ, LIVING_MAP_AUTO_EXPLORE="false")
command = [str(root / "scripts/run_demo.sh"), "--speed", speed]
if mode == "guided":
    command.append("--fallback")
with (root / "runtime/demo.log").open("w") as log:
    process = subprocess.Popen(command, cwd=root, env=env, stdin=subprocess.DEVNULL,
                               stdout=log, stderr=log, start_new_session=True)
print(f"Replay launched pid={process.pid} mode={mode} speed={speed}", flush=True)
