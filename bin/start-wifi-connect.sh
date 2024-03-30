#!/bin/bash

# Wait for 30 seconds to ensure that any network services have enough time to initialize before checking for a default gateway.
# This can be particularly useful in environments where network connections take a moment to establish after boot.
sleep 30

# Check for a default gateway
if ip route | grep -q default; then
    printf 'Default gateway found. Skipping WiFi Connect.\n'
else
    printf 'No default gateway found. Starting WiFi Connect.\n'
    sudo systemctl stop apache2.service
    CURRENT_TIME=$(who -b | awk '{print $3 " " $4}' | xargs -I {} date -d {} +"%Y-%m-%d-%H-%M")
    wifi-connect -s radxa-${CURRENT_TIME} -g 192.168.4.1 -d 192.168.4.2,192.168.4.5
fi
