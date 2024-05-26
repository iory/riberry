#!/bin/bash

# Wait for 30 seconds to ensure that any network services have enough time to initialize before checking for a default gateway.
# This can be particularly useful in environments where network connections take a moment to establish after boot.
sleep 30
iwgetid -r

# Check for a default gateway
if [ $? -eq 0 ]; then
    printf 'Skipping WiFi Connect.\n'
else
    printf 'No default gateway found. Starting WiFi Connect.\n'
    MAC_ADDRESS=$(cat /sys/class/net/wlan0/address | tr -d ':')
    wifi-connect -s radxa-${MAC_ADDRESS} -g 192.168.4.1 -d 192.168.4.2,192.168.4.5
fi
