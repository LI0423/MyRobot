#!/bin/bash
set -e

# Data
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ros2_ws"
START_SCRIPT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/start_robot.py"

echo "=================================================="
echo "          XiaoZhi Robot - Build & Run             "
echo "=================================================="
echo "Workspace: $WORKSPACE_DIR"

# 1. Source ROS 2 Environment (Try multiple paths)
if [ -f "/opt/ros/humble/setup.zsh" ]; then
    source /opt/ros/humble/setup.zsh
elif [ -f "/opt/ros/foxy/setup.zsh" ]; then
    source /opt/ros/foxy/setup.zsh
elif [ -f "/opt/ros/rolling/setup.zsh" ]; then
    source /opt/ros/rolling/setup.zsh
else
    echo "⚠️  ROS 2 setup.zsh not found in standard locations."
    echo "Assuming environment is already set up."
fi

# 2. Install Dependencies (Optional, skippable with --skip-deps)
if [[ "$1" != "--skip-deps" ]]; then
    echo "--> Checking dependencies..."
    cd "$WORKSPACE_DIR"
    # rosdep install --from-paths src --ignore-src -r -y
    echo "Skipping rosdep for local dev (uncomment in script if needed)"
fi

# 3. Build Workspace
echo "--> Building workspace..."
cd "$WORKSPACE_DIR"
colcon build --symlink-install

# 4. Source Local Overlay
echo "--> Sourcing local setup..."
source install/setup.zsh

# 5. Run Robot Starter
echo "--> Starting Robot System..."
# Navigate back to root to find start_robot.py correctly module-wise if needed
cd ..
python3 "$START_SCRIPT" "${@:1}"
