#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/_env.sh"
# Ask the command post to brief the rescue robots: it decides WHO goes (fire robot
# for fires, rescue robot for victims) and WHAT the targets are, in priority order.
# Each robot decides how to get there.
curl -fsS -X POST http://localhost:8081/api/demo -H 'content-type: application/json' \
  -d '{"action":"dispatch"}' | python -m json.tool
