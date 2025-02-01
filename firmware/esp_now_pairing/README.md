# ESP-Now pairing

Send the following data to pair two linux machines

- IP address

# Usage

1. Burn firmware to pairing devices

```
# For sender device.
# This device is usually connected to ROS master.
pio run -t upload -e main

# For receiver device.
# This device is usually connected to ROS client.
pio run -t upload -e secondary
```

2. Start pairing program in each host computer

```
rosrun riberry_startup esp_now_pairing_single.py
```

3. Connect pairing devices
