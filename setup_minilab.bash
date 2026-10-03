#!/usr/bin/env bash

# ==============================================================
# EE5112 MiniLab 1.2 environment
# ==============================================================

MINILAB_ROOT="$HOME/EE5112_MiniLab1.2"
MINILAB_INSTALL="$MINILAB_ROOT/install"
VEHICLE_PREFIX="$MINILAB_INSTALL/ee5112_vehicle"


# ROS 2 Humble
source /opt/ros/humble/setup.bash


# Colcon-generated workspace environment
if [ -f "$MINILAB_INSTALL/setup.bash" ]; then
    source "$MINILAB_INSTALL/setup.bash"
fi


# Workaround for isolated ee5112_vehicle prefix not being added
# automatically by the generated workspace setup.
if [ -d "$VEHICLE_PREFIX" ]; then

    export AMENT_PREFIX_PATH="$VEHICLE_PREFIX:$AMENT_PREFIX_PATH"

    export PATH="$VEHICLE_PREFIX/lib/ee5112_vehicle:$PATH"

    # Load the generated Python-path hooks.
    if [ -f "$VEHICLE_PREFIX/share/ee5112_vehicle/package.bash" ]; then
        source "$VEHICLE_PREFIX/share/ee5112_vehicle/package.bash"
    fi

else

    echo "[EE5112] ee5112_vehicle has not been built."
    echo "[EE5112] Run:"
    echo
    echo "  cd ~/EE5112_MiniLab1.2"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  colcon build --base-paths ros2_ws/src --symlink-install"

    return 1 2>/dev/null || exit 1

fi


echo "[EE5112] Environment ready."
echo "[EE5112] Package prefix: $VEHICLE_PREFIX"
