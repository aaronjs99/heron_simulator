#!/bin/bash
set -e

# Configuration
RECORDING_DIR="/home/ig-handle/.gemini/antigravity/brain/e97c5ab2-694d-4fc0-b7b0-aede8db24f1d"
BAG_NAME="inspection_mission.bag"
VIDEO_NAME="inspection_mission.mp4"
DURATION=60 # Seconds to run

# Cleanup function
cleanup() {
    echo "Stopping recording..."
    kill $FFMPEG_PID 2>/dev/null || true
    kill $ROSBAG_PID 2>/dev/null || true
    echo "Stopping simulation..."
    rosnode kill -a
    killall -9 gzserver gzclient rosmaster Xvfb 2>/dev/null || true
}
trap cleanup EXIT

echo "Starting Xvfb..."
# Start virtual display :99
Xvfb :99 -screen 0 1280x720x24 &
export DISPLAY=:99
sleep 3

echo "Cleaning up Viz Ports (8080-8089)..."
for port in {8080..8089}; do
  fuser -k -n tcp $port 2>/dev/null || true
done
sleep 1

echo "Starting Simulation..."
source /home/ig-handle/catkin_ws/devel/setup.bash
roslaunch heron_simulator simulation_full.launch gui:=false &
SIM_PID=$!
sleep 15 # Wait for Gazebo to load

echo "Starting Video Recording..."
ffmpeg -y -f x11grab -s 1280x720 -r 30 -i :99 -c:v libx264 -preset fast -pix_fmt yuv420p "$RECORDING_DIR/$VIDEO_NAME" > /dev/null 2>&1 &
FFMPEG_PID=$!

echo "Starting Rosbag Recording..."
cd "$RECORDING_DIR"
rosbag record -O "$BAG_NAME" -a -x "(.*)camera(.*)" --duration=$DURATION &
ROSBAG_PID=$!

echo "Settling..."
sleep 5

echo "Triggering Mission: Inspect Pillar 2..."
rostopic pub -1 /oracle/query/voice std_msgs/String "data: 'inspect pillar 2'"

echo "Waiting for mission to execute ($DURATION seconds)..."
sleep $DURATION

echo "Mission Time Complete."
