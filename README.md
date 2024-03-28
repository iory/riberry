# Riberry: Robot Intelligent Binary Enhancement Resource Revolutionary Yield

## Install

### For radxa

After cloning this repository into your catkin workspace, build it:

```
catkin build riberry_startup
```

After sourcing, install the systemd-related programs with the following commands:

```
sudo ./install.py
sudo reboot
```

You can use the dry-run option to see which scripts will be installed without actually installing them:

```
sudo ./install.py --dry-run
```

### For atom s3 and atom echo

Please write the programs found under the firmware section into the atom s3 and atom echo to display the IP and capture audio.
