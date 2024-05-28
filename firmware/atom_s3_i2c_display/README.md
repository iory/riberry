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
