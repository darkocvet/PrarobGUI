#!/bin/bash

# IMPORTANT: You must run this script using the `source` command so it affects your current shell!
# Usage: source activate.sh

echo "=== Activating PRAROB Environment ==="

# 1. Source the global ROS 2 installation
# (Checks for Jazzy, Humble, Iron, and Foxy)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[OK] Sourced ROS 2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[OK] Sourced ROS 2 Humble"
elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
    echo "[OK] Sourced ROS 2 Iron"
elif [ -f /opt/ros/foxy/setup.bash ]; then
    source /opt/ros/foxy/setup.bash
    echo "[OK] Sourced ROS 2 Foxy"
else
    echo "[WARNING] Could not find a standard ROS 2 installation in /opt/ros/"
fi

# 2. Source the local colcon workspaces
workspaces=(
    "install/setup.bash"
    "resources/install/setup.bash"
    "resources/prarob_interact/install/setup.bash"
    "resources/finalrviz/Robotska_description/install/setup.bash"
    "resources/prarob_yolo/install/setup.bash"
    "resources/prarob_yolo/yolo_ros/install/setup.bash"
)

found_ws=false
for ws in "${workspaces[@]}"; do
    if [ -f "$ws" ]; then
        source "$ws"
        echo "[OK] Sourced local colcon workspace ($ws)"
        found_ws=true
    fi
done

if [ "$found_ws" = false ]; then
    echo "[INFO] No local 'install/setup.bash' found in known subdirectories. Make sure you ran 'colcon build' first."
fi

# 3. Load environment variables from the .env file
if [ -f "resources/.env" ]; then
    # set -a automatically exports all variables defined until set +a
    set -a
    source resources/.env
    set +a
    echo "[OK] Exported environment variables from resources/.env (OPENAI_API_KEY, etc.)"
fi

# 4. Helper aliases
alias run_yolo="ros2 launch yolo_bringup yolo.launch.py use_tracking:=False device:=cpu"
echo "[OK] Added 'run_yolo' alias to easily start the camera & YOLO node (tracking disabled to prevent crashes)"

echo "====================================="
echo "Environment is ready! You can now run your nodes or the GUI in separate terminals:"
echo "  1) python3 gui.py (The main GUI interface)"
echo "  2) run_yolo       (Starts the camera and YOLO detections)"
echo "  3) ros2 run prarob_interact nl_agent (Starts the autonomous AI brain)"
