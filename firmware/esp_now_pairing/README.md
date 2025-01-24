# ESP-Now pairing

Send the following data to pair two linux machines

- IP address

# Usage

1. Burn firmware to pairing devices

```
# For sender device.
# This device is usually connected to ROS master.
PAIRING_TYPE=SENDER pio run -t upload

# For receiver device.
# This device is usually connected to ROS client.
PAIRING_TYPE=RECEIVER pio run -t upload
```

2. Start pairing program in each host computer

```
rosrun riberry_startup esp_now_pairing.py
```

3. Connect pairing devices
