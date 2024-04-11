# atom_echo_i2c_audio

This Arduino program enables an ESP32-based M5Atom device to function as both an I2S master for audio data capture through a microphone and an I2C slave for sending this data to an I2C master. Additionally, it can receive RGB values via I2C to control the device's onboard LED.

## Functionality

- Captures audio via an I2S microphone.
- Sends audio data to an I2C master.
- Receives RGB values from an I2C master to control the LED display.

## Hardware Requirements

- ESP32-based M5Atom Controller
- I2S microphone
- I2C Master device

## Write Firmware via PlatformIO

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

### Compile and write firmware to atom echo

```
pio run -t upload
```
