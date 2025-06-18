#!/bin/bash

PYTHON_EXECUTABLE="/usr/bin/python3"

if [ -f /etc/os-release ]; then
    . /etc/os-release
    VERSION_MAJOR=$(echo "$VERSION_ID" | cut -d'.' -f1)
    if [ "$VERSION_MAJOR" -ge 24 ]; then
        export PYENV_ROOT="$HOME/.python3_venv"
        export PATH="$PYENV_ROOT/bin:$PATH"

        if command -v pyenv 1>/dev/null 2>&1; then
            eval "$(pyenv init -)"
        fi
        echo $PATH
        PYTHON_EXECUTABLE="python3"
    fi
fi

if [ -f /opt/ros/one/setup.bash ]; then
    . /opt/ros/one/setup.bash
elif [ -f /opt/ros/noetic/setup.bash ]; then
    . /opt/ros/noetic/setup.bash
fi

# Export ROS_MASTER_URI after /opt/ros/.../setup.bash
# because ROS_MASTER_URI is set to localhost by the script.
if [ -n "$RIBERRY_ROS_MASTER_IP" ]; then
    export ROS_MASTER_URI=http://$RIBERRY_ROS_MASTER_IP:11311
fi

exec "$PYTHON_EXECUTABLE" /usr/local/bin/display_information.py
