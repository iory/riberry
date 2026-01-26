# Atomic Robot Tools

## Set baud rate and ID to Dynamixel motor

```
pio run -t upload -e dynamixel_setBaudandID
```

## Control Dynamixel motor via AtomS3 monitor clicks

```
pio run -t upload -e dynamixel_safe_test
```

## Control Dynamixel motor via ESPNOW communicaation

1. Burn firmware to pairing devices

    ```
    # For main (sender) device. This device is usually connected to ROS computer.
    pio run -t upload -e espnow_dynamixel_controller_main

    # For receiver (secondary) device. This device is usually connected to atomic tools.
    pio run -t upload -e espnow_dynamixel_controller_secondary
    ```

2. Start the pairing program on the computer connected to the main (sender) device

    To protect the motor, Stop command is automatically sent 5 seconds after sending a Forward/Reverse command.

    ```
    rosrun riberry_startup espnow_dynamixel_controller_node.py
    ```

3. Send control command via rostopic

    ```
    rostopic pub /motor_command  std_msgs/String "data: 'Forward'"
    rostopic pub /motor_command  std_msgs/String "data: 'Stop'"
    rostopic pub /motor_command  std_msgs/String "data: 'Reverse'"
    ```

## Control Dynamixel motor via WiFi

1. Burn firmware to device

```
pio run -e wifi_dynamixel_controller -t upload
```

2. Run rosserial (rosserial_python is unstable)

```
rosrun rosserial_server socket_node
```
