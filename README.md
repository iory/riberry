# Riberry: Robot Intelligent Binary Enhancement Resource Revolutionary Yield

## Install Riberry

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
./install.py --dry-run
```

### For atom s3 and atom echo

Please write the programs found under the [firmware](https://github.com/iory/riberry/tree/master/firmware) section into the atom s3 and atom echo to display the IP and capture audio.

## Distribute radxa image as SD card

When distributing images, it's necessary to adjust the disk size among other parameters initially due to writing to an SD card.
The `resize-helper.service` is a tool designed for this purpose. To enable this, please use the `--enable-oneshot` option to activate these settings.
Additionally, the `change-hostname-helper.service`, which randomly changes the hostname, will also be enabled.

Run the following command just before making SD cards for distribution:

```
sudo ./install.py --enable-oneshot
```

If you are concerned about the security of the network:
```
# sudo rm -f /etc/NetworkManager/system-connections/*
```
