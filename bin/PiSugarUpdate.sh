#!/bin/bash

set -e

TEMPDIR=$(mktemp -d /tmp/pisugar-update.XXXXXXX)
mkdir -p $TEMPDIR

function cleanup() {
    rm -rf $TEMPDIR
}
trap cleanup ERR


# Path to the model file
file_path="/proc/device-tree/model"

# Read the model from the file
model=$(cat "$file_path")

if which dpkg > /dev/null; then
    # Download fireware and programmer
    wget -O $TEMPDIR/pisugar-3-application.bin https://cdn.pisugar.com/release/PiSugar3Firmware/fm33lc023n/pisugar-3-application.bin

    ### Enable case insensitive matching
    shopt -s nocasematch

    # Install programmer
    if ! which pisugar-programmer > /dev/null; then
        if [[ "$model" == *raspberry* ]]; then
            wget -O $TEMPDIR/pisugar-programmer_1.6.4_arm64.deb https://github.com/PiSugar/pisugar-power-manager-rs/releases/download/v1.6.4/pisugar-programmer_1.6.4_arm64.deb
            sudo dpkg -i $TEMPDIR/pisugar-programmer_1.6.4_arm64.deb
        elif [[ "$model" == *radxa* ]]; then
            wget -O $TEMPDIR/pisugar-programmer_1.6.4_armhf.deb https://cdn.pisugar.com/release/pisugar-programmer_1.6.4_armhf.deb
            sudo dpkg -i $TEMPDIR/pisugar-programmer_1.6.4_armhf.deb
        fi
    fi

    # Upgrade firmware

    if [[ "$model" == *raspberry* ]]; then
        echo y | pisugar-programmer -r $TEMPDIR/pisugar-3-application.bin
    elif [[ "$model" == *radxa* ]]; then
        echo y | pisugar-programmer -b 3 -r $TEMPDIR/pisugar-3-application.bin
    fi

    ### Disable case insensitive matching
    shopt -u nocasematch

    # Wait until pisugar is ready
    echo "Wait for 10 seconds"
    sleep 10

    # Upgrade success
    echo "Upgrade complete!"
else
    echo "You need to manually download the fireware and upgrade the pisugar: "
    echo "Fireware url: https://cdn.pisugar.com/release/PiSugar3Firmware/fm33lc023n/pisugar-3-application.bin"
    echo "Programmer url: https://cdn.pisugar.com/release/pisugar-programmer_1.6.4_armhf.deb"
fi

echo "If you need help, visit https://github.com/PiSugar/"

cleanup
