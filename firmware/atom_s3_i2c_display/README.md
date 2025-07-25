# Atom S3 I2C Display

This is firmware that displays information on atom s3 via i2c.

### Prerequirements

Install udev to give permission to the device.

```
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Ubuntu/Debian users may need to add own “username” to the “dialout” group if they are not “root”, doing this issuing

```
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER
```

```
pip3 install platformio -U
```

### Compile and write firmware to atom s3

You can flash firmware directly from your web browser via USB connection at: https://iory.github.io/riberry/

Alternatively, you can compile and upload using PlatformIO:

```
pio run -t upload
```

If you are using Grove cables, set the `USE_GROVE` environment variable to `1` before running the command:

```
USE_GROVE=1 pio run -t upload
```

Setting `PRINT_CPU_USAGE=1` will display the CPU usage via USB Serial.

```
PRINT_CPU_USAGE=1 pio run -t upload
```

After that,

```
pio device monitor
Core 0:
  CommunicationBase         : 66.92%
  ButtonManager             : 0.04%
  Mode Manager              : 0.01%
  Idle CPU Usage            : 33.03%

Core 1:
  Pairing                   : 0.00%
  DisplayInformationMode    : 0.00%
  DisplayQRcodeMode         : 0.00%
  PairingMode               : 0.00%
  SystemDebugMode           : 0.02%
  Idle CPU Usage            : 99.98%
```

If you want to set the LCD rotation, set the `LCD_ROTATION` environment variable to `0`, `1`, `2`, or `3` before running the command:

```
LCD_ROTATION=1 pio run -t upload
```

If you want to set the I2C address, set the `I2C_ADDR` environment variable to the desired address (in hexadecimal format) before running the command:

```
I2C_ADDR=0x42 pio run -t upload
```

If you want to use USB serial communication, set the `USE_USB_SERIAL` environment variable to `1` before running the command:

```
USE_USB_SERIAL=1 pio run -t upload
```



If you want to use M5Stack instead of AtomS3:

```
pio run -t upload -e m5stack-basic
```

`Main` and `Secondary` Option.

```
PAIRING_TYPE=Main USE_USB_SERIAL=1 pio run -t upload
```

```
PAIRING_TYPE=Secondary USE_USB_SERIAL=1 pio run -t upload
```

### Upload via ssh

We can upload firmware via ssh.

```
pio run
pio run --upload-port scp:rock@<TARGET_IP>
```

## Control multiple modes

The `atom_s3_i2c_display` includes multiple modes. This feature allows the robot to be controlled using both the `/i2c_button_state` and `/i2c_mode topics`, making control programming easier.

### Program Structure

- The AtomS3 firmware is paired with a corresponding rospy program on the computer. To use any mode, first update the AtomS3 firmware, then launch the matching rospy node.

| AtomS3 Firmware | Computer rospy Node |
| ------------- | ------------- |
| display_information_mode.cpp | display_information.py |
| display_qr_code_mode.cpp | display_information.py |
| servo_control_mode.cpp | servo_control_mode.py |
| pressure_control_mode.cpp | pressure_control_mode.py |
| teaching_mode.cpp | teaching_mode.py |
| display_battery_graph_mode.cpp | display_battery_graph_mode.py |

- `i2c_button_state_publisher.cpp` facilitates communication between these programs.
The following sections provide details on the base code for the AtomS3 firmware.

- `main.cpp` includes instances for each mode and stores them in a list. Each time a long click occurs, the active task switches to another mode. Note that this program does not include instances of Servo Control Mode, Pressure Control Mode and Teachiing Mode by default.

- `primitive_lcd.h` is a shared library for the AtomS3, containing functions for drawing on the LCD and holding variables needed for rendering data, such as jpegBuf and qrCodeData.

- `communication_base.h` is a shared library for I2C communication with the AtomS3. In the receiveEvent function, data is received from the I2C master and stored in the PrimitiveLCD class object. The requestEvent function sends button state and mode information to the I2C master.

- `button_manager.h` is a shared library that detects which button was pressed and identifies the type of press.

### Mode Descriptions

- Display Information Mode

  Displays the device's IP address and battery level.

- Display QR Code Mode

  Shows a QR code on the screen.

- Servo Control Mode

  Controls the Kondo servo. A single click toggles the servo on or off.

- Pressure Control Mode

  Controls pressure by toggling the pump and solenoid valve. The current pressure value is shown on the AtomS3 LCD. A single click toggles the vacuum on or off.

- Teaching Mode

  This mode includes three states: `WAIT`, `RECORD`, and `PLAY`.

  In the `WAIT` state:
  - A single click transitions to the `RECORD` state.
  - A double click transitions to the `PLAY` state.
  - A triple click turns off the servo.

  In the `RECORD` state:
  - Recording motion automatically starts at the moment of transition from `WAIT` state to `RECORD` state
  - A single click stops recording motion.
  - After recording, the state automatically returns to `WAIT`.

  In the `PLAY` state:
  - A single click toggles the selected motion. The current selection is indicated by green.
  - A double click starts playing the selected motion.
    - After starting, a double click stops playing motion.
  - A triple click deletes the selected motion.
  - After playback or delete, the state automatically returns to `WAIT`.

- Display Battery Graph Mode

  Displays the remaining battery charge as a percentage, with a graph of the change over time underneath.

### How to Add New Modes

1. **Define a New Mode**

   - **Firmware**
     - Add 2 entries in [`mode_type.h`](https://github.com/iory/riberry/blob/main/firmware/atom_s3_i2c_display/lib/mode/mode_type.h)
     - Add 3 entries in [`main.cpp`](https://github.com/iory/riberry/blob/main/firmware/atom_s3_i2c_display/src/main.cpp)

   - **Python**
     - Add 2 entries in [`mode_type.py`](https://github.com/iory/riberry/blob/main/riberry/mode_type.py)

2. **Define a New Packet for the Mode**

   - **Firmware**
     - Add 1 entry in [`communication_base.cpp`](https://github.com/iory/riberry/blob/main/firmware/atom_s3_i2c_display/lib/com/communication_base.cpp)
     - Add 1 entry in [`packet.h`](https://github.com/iory/riberry/blob/main/firmware/atom_s3_i2c_display/lib/com/packet.h)

   - **Python**
     - Add 1 entry in [`base.py`](https://github.com/iory/riberry/blob/main/riberry/com/base.py)

3. **Add a Program to Handle the New Mode**

   For example, place your `rospy` node under [`node_scripts`](https://github.com/iory/riberry/blob/main/ros/riberry_startup/node_scripts/)

4. **Add the Mode to the roslaunch File for convenience**

   Add 2 entries in [`all_modes.launch`](https://github.com/iory/riberry/blob/main/ros/riberry_startup/launch/all_modes.launch)

5. **(Optional) Define New Mode Firmware**

   If you want a custom visualization for the new mode, place the firmware in [`lib`](https://github.com/iory/riberry/blob/main/firmware/atom_s3_i2c_display/lib/)

See mode addition sample Pull Request for reference:
[PR #303](https://github.com/iory/riberry/pull/303)
