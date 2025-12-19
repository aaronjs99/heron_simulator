#!/bin/bash
# heron_simulation/scripts/kill_sim.sh
# Kill all ROS and Gazebo processes for a clean restart

echo "Killing all ROS and Gazebo processes..."

# Kill by process name
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f rosmaster
pkill -9 -f roscore
pkill -9 -f roslaunch
pkill -9 -f rosout

# Kill common ROS nodes that might be zombies
pkill -9 -f ekf_localization_node
pkill -9 -f navsat_transform_node
pkill -9 -f robot_state_publisher
pkill -9 -f move_base

# Wait for processes to terminate
sleep 2

# Verify
remaining=$(pgrep -c -f "gzserver|rosmaster|roscore" 2>/dev/null || echo "0")
if [ "$remaining" -gt 0 ]; then
    echo "Warning: Some processes may still be running. Attempting force kill..."
    pkill -9 -f "gzserver|rosmaster|roscore"
    sleep 1
fi

echo "All ROS/Gazebo processes killed. Ready to launch."
