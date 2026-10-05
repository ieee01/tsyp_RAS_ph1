#!/usr/bin/env bash
set -euo pipefail
export LIVING_MAP_SKIP_WORKSPACE_SETUP=1
source "$(dirname "$0")/_env.sh"
unset LIVING_MAP_SKIP_WORKSPACE_SETUP
command -v colcon >/dev/null || { echo "colcon not installed"; exit 2; }

# Deterministic default: do not let an old isolated/shared-folder install poison
# package discovery. Set LIVING_MAP_INCREMENTAL=1 only for local development.
if [[ "${LIVING_MAP_INCREMENTAL:-0}" != "1" ]]; then
  echo "[LivingMap] Clean merged build"
  rm -rf build install log
fi

colcon build --merge-install --event-handlers console_direct+
