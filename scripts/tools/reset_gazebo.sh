#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   reset_gazebo.sh                  # safe: Gazebo only + free Gazebo ports
#   reset_gazebo.sh --full           # also kill ROS core/launch (nuclear)
#   reset_gazebo.sh --port 11345     # override gazebo master port
PORT=11345
FULL=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --full) FULL=1; shift ;;
    --port) PORT="$2"; shift 2 ;;
    *) echo "Unknown arg: $1" >&2; exit 2 ;;
  esac
done

echo "[reset_gazebo] Cleaning Gazebo (PORT=${PORT}, FULL=${FULL})"

# Free Gazebo ports (master often PORT, server often PORT+1)
for p in "${PORT}" "$((PORT+1))"; do
  if ss -ltn "sport = :${p}" | tail -n +2 | grep -q .; then
    echo "[reset_gazebo] Port ${p} in use. Killing listeners on ${p}/tcp ..."
    sudo fuser -k "${p}/tcp" || true
  fi
done

# Kill Gazebo processes
pkill -f gzserver || true
pkill -f gzclient || true
pkill -f gazebo   || true

if [[ "${FULL}" -eq 1 ]]; then
  echo "[reset_gazebo] FULL requested: killing ROS core/launch and common nodes"
  pkill -f rosmaster || true
  pkill -f roscore   || true
  pkill -f roslaunch || true
  pkill -f rosout    || true
  pkill -f ekf_localization_node || true
  pkill -f navsat_transform_node || true
  pkill -f robot_state_publisher || true
  pkill -f move_base || true
fi

sleep 1

echo "[reset_gazebo] Remaining (best-effort):"
pgrep -a gzserver gzclient gazebo rosmaster roscore roslaunch 2>/dev/null || true

echo "[reset_gazebo] Done."