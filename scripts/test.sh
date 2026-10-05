#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/_env.sh"
python3 -m pytest -q tests
if command -v colcon >/dev/null && [[ -f /opt/ros/jazzy/setup.bash ]] && grep -q '<test_depend>' src/*/package.xml; then
  colcon test --merge-install --event-handlers console_direct+
  colcon test-result --verbose
elif command -v colcon >/dev/null && [[ -f /opt/ros/jazzy/setup.bash ]]; then
  echo "No package-local ROS tests declared; central pytest suite completed."
else
  echo "ROS/colcon absent: ROS package tests skipped; pure-Python tests completed."
fi
