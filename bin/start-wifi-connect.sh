#!/bin/bash

# Wait for 30 seconds to ensure that any network services have enough time to initialize before checking for a default gateway.
# This can be particularly useful in environments where network connections take a moment to establish after boot.
sleep 30

# Get the SSID of the connected WiFi network (if any)
SSID=$(iwgetid -r)

# Path to the model file
file_path="/proc/device-tree/model"

# Read the model from the file
model=$(cat "$file_path")

# Enable case insensitive matching
shopt -s nocasematch

# Determine the device model
if [[ "$model" == *raspberry* ]]; then
    MODEL="raspi"
elif [[ "$model" == *radxa* ]]; then
    MODEL="radxa"
elif [[ "$model" == "Khadas VIM4" ]]; then
    MODEL="vim4"
else
    MODEL="unknown"
fi

# Disable case insensitive matching
shopt -u nocasematch

# Check if the SSID is obtained (indicating a successful WiFi connection)
if [ -n "$SSID" ]; then
    printf 'Skipping WiFi Connect. Connected to SSID: %s\n' "$SSID"
else
    printf 'No default gateway found. Starting WiFi Connect.\n'
    MAC_ADDRESS=$(cat /sys/class/net/wlan0/address | tr -d ':')
    wifi-connect -s "${MODEL}-${MAC_ADDRESS}" -g 192.168.4.1 -d 192.168.4.2,192.168.4.5

    # Check if wifi-connect command was successful
    if [ $? -eq 0 ]; then
        printf 'WiFi Connect started successfully.\n'
    else
        printf 'WiFi Connect failed to start.\n'
    fi
fi
