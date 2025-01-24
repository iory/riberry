#!/bin/bash

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

/usr/bin/python3 /usr/local/bin/display_information.py
