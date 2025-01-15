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

```
pio run -t upload
```

If you are using Grove cables, set the `USE_GROVE` environment variable to `1` before running the command:

```
USE_GROVE=1 pio run -t upload
```

If you want to set the LCD rotation, set the `LCD_ROTATION` environment variable to `0`, `1`, `2`, or `3` before running the command:

```
LCD_ROTATION=1 pio run -t upload
```

If you want to set the I2C address, set the `I2C_ADDR` environment variable to the desired address (in hexadecimal format) before running the command:

```
I2C_ADDR=0x42 pio run -t upload
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

- `atom_s3_lcd.h` is a shared library for the AtomS3, containing functions for drawing on the LCD and holding variables needed for rendering data, such as jpegBuf and qrCodeData.

- `atom_s3_i2c.h` is a shared library for I2C communication with the AtomS3. In the receiveEvent function, data is received from the I2C master and stored in the AtomS3LCD class object. The requestEvent function sends button state and mode information to the I2C master.

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
