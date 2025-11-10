#!/bin/bash

PYTHON_EXECUTABLE="/usr/bin/python3"

if [ -f /etc/os-release ]; then
    . /etc/os-release
    VERSION_MAJOR=$(echo "$VERSION_ID" | cut -d'.' -f1)
    if [ "$VERSION_MAJOR" -ge 24 ]; then
        export RIBERRY_VENV="$HOME/.riberry_venv"
        if [ -d "$RIBERRY_VENV" ]; then
            export PATH="$RIBERRY_VENV/bin:$PATH"
            PYTHON_EXECUTABLE="python3"
        fi
    fi
fi

# ROS2 setup
if [ -f /opt/ros/jazzy/setup.bash ]; then
    . /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    . /opt/ros/humble/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    . /opt/ros/iron/setup.bash
# ROS1 setup
elif [ -f /opt/ros/one/setup.bash ]; then
    . /opt/ros/one/setup.bash
elif [ -f /opt/ros/noetic/setup.bash ]; then
    . /opt/ros/noetic/setup.bash
fi

# Export ROS_MASTER_URI after /opt/ros/.../setup.bash (ROS1 only)
# because ROS_MASTER_URI is set to localhost by the script.
if [ -n "$RIBERRY_ROS_MASTER_IP" ]; then
    export ROS_MASTER_URI=http://$RIBERRY_ROS_MASTER_IP:11311
fi

exec "$PYTHON_EXECUTABLE" /usr/local/bin/display_information.py
