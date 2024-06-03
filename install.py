#!/usr/bin/env python3

import os
import os.path as osp

import subprocess
import sys
import argparse

def is_regular_file(filename):
    return not filename.endswith('~') and not filename.startswith('.')


def identify_device():
    try:
        with open('/proc/cpuinfo', 'r') as f:
            cpuinfo = f.read()

        if 'Raspberry Pi' in cpuinfo:
            return 'Raspberry Pi'

        with open('/proc/device-tree/model', 'r') as f:
            model = f.read().strip()

        if 'Radxa' in model or 'ROCK Pi' in model:
            return model

        return 'Unknown Device'
    except FileNotFoundError:
        return 'Unknown Device'


def create_symlinks(source_dir, target_dir, dry_run=False):
    added_symlinks = []
    for item in os.listdir(source_dir):
        source_path = os.path.join(source_dir, item)
        if not (osp.isfile(source_path) and
                is_regular_file(item)):
            continue
        target_path = os.path.join(target_dir, item)
        if os.path.isfile(source_path):
            if dry_run:
                print(f"Dry-run: Would create symlink: {target_path} -> {source_path}")
            else:
                if os.path.lexists(target_path):
                    os.remove(target_path)
                os.symlink(osp.abspath(source_path), osp.abspath(target_path))
                print(f"Created symlink: {target_path} -> {source_path}")
                added_symlinks.append(target_path)
    return added_symlinks


def copy_files(source_dir, target_dir, dry_run=False):
    for item in os.listdir(source_dir):
        if not is_regular_file(item):
            continue
        source_path = os.path.join(source_dir, item)
        target_path = os.path.join(target_dir, item)
        if os.path.isfile(source_path):
            if dry_run:
                print(f"Dry-run: Would copy {source_path} to {target_dir}")
            else:
                subprocess.run(["cp", "-f", source_path, target_dir])
                print(f"Copied {item} to {target_dir}")


def enable_systemd_services(symlinks, dry_run=False):
    for symlink_path in symlinks:
        item = os.path.basename(symlink_path)
        if item.endswith('.service'):
            if dry_run:
                print(f"Dry-run: Would enable systemd service: {item}")
            else:
                subprocess.run(["systemctl", "enable", item])
                print(f"Enabled systemd service: {item}")


def execute_dtc_command(dry_run, output_path, source_dts):
    if dry_run:
        print(f"Dry-run: Would execute 'dtc' command to create device tree blob at '{output_path}' from '{source_dts}'.")
    else:
        dtc_command = ['dtc', '-@', '-I', 'dts', '-O', 'dtb', '-o', output_path, source_dts]
        subprocess.run(dtc_command)
        print(f"Executed 'dtc' command to create device tree blob at '{output_path}' from '{source_dts}'.")


def main(dry_run=False, enable_oneshot=False):
    if dry_run is False and os.geteuid() != 0:
        print("This script must be run as root.")
        sys.exit(1)

    bin_source_dir = './bin'
    systemd_source_dir = './systemd'
    bin_target_dir = '/usr/local/bin'
    systemd_target_dir = '/etc/systemd/system'

    copy_files('./boot', '/boot', dry_run=dry_run)
    create_symlinks(bin_source_dir, bin_target_dir, dry_run=dry_run)

    if dry_run is False:
        os.makedirs('/usr/local/dict', 0o777, exist_ok=True)
    create_symlinks('./dict', '/usr/local/dict', dry_run=dry_run)

    if dry_run is False:
        os.makedirs('/etc/opt/riberry', 0o777, exist_ok=True)
    create_symlinks('./etc/opt/riberry', '/etc/opt/riberry', dry_run=dry_run)

    added_symlinks = create_symlinks('./ros/riberry_startup/systemd',
                                     systemd_target_dir, dry_run=dry_run)
    added_symlinks += create_symlinks(systemd_source_dir, systemd_target_dir, dry_run=dry_run)
    if enable_oneshot:
        added_symlinks += create_symlinks(
            './systemd/oneshot', '/etc/systemd/system', dry_run=dry_run)

    enable_systemd_services(added_symlinks, dry_run=dry_run)

    if identify_device() == 'Radxa Zero':
        execute_dtc_command(
            dry_run,
            '/boot/dtbs/5.10.69-12-amlogic-g98700611d064/amlogic/overlay/meson-g12a-i2c-ee-m1-gpioh-6-gpioh-7.dtbo',
            './overlays/i2c1.dts'
        )
        execute_dtc_command(
            dry_run,
            '/boot/dtbs/5.10.69-12-amlogic-g98700611d064/amlogic/overlay/meson-g12a-i2c-ee-m3-gpioa-14-gpioa-15.dtbo',
            './overlays/i2c3.dts'
        )
        execute_dtc_command(
            dry_run,
            '/boot/dtbs/5.10.69-12-amlogic-g98700611d064/amlogic/overlay/meson-g12a-gpio-line-names.dtbo',
            './overlays/meson-g12a-gpio-line-names.dts'
        )

    if dry_run:
        print("Dry-run: No install python libraries.")
    else:
        username = os.getlogin()
        command = f'sudo -u {username} pip3 install -r requirements.txt'
        subprocess.run(command, shell=True)

    if dry_run:
        print("Dry-run mode: No changes were made.")
    else:
        print("Installation completed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Install script with dry-run option.")
    parser.add_argument("--dry-run", action="store_true", help="Run the script in dry-run mode to display the operations without making any changes.")
    parser.add_argument("--enable-oneshot", action="store_true",
                        help="Enable the oneshot services like a resize-helper and change-hostname-helper.")

    args = parser.parse_args()
    main(args.dry_run, args.enable_oneshot)
