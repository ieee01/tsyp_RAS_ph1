#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/_env.sh"
ros2 service call /living_map/writer/fail std_srvs/srv/Trigger "{}"
