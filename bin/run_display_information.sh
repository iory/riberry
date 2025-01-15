#!/bin/bash

if [ -f /opt/ros/noetic/setup.bash ]; then
    . /opt/ros/noetic/setup.bash
fi

/usr/bin/python3 /usr/local/bin/display_information.py
