#!/bin/bash
# heron_simulation/scripts/launch_sim.sh
# Launch simulation with guaranteed clean start

set -e

echo "=== Cleaning up previous processes ==="
pkill -9 -f "gzserver|gzclient|rosmaster|roscore|roslaunch|rosout" 2>/dev/null || true
killall -9 gzserver gzclient rosmaster roscore roslaunch python python3 2>/dev/null || true
sleep 2

# Verify all cleaned
remaining=$(pgrep -c -f "gzserver|rosmaster|roscore" 2>/dev/null || echo "0")
if [ "$remaining" -gt 0 ] 2>/dev/null; then
    echo "Warning: Force killing remaining processes..."
    pkill -9 -f "gzserver|rosmaster|roscore" 2>/dev/null || true
    sleep 1
fi

echo "=== Starting simulation ==="
roslaunch heron_simulation simulation_full.launch "$@"

