#!/usr/bin/env python3

import argparse
import getpass
import os
import os.path as osp
import subprocess
import sys


def find_broken_symlinks(directory):
    broken_links = []
    for root, _, files in os.walk(directory):
        for filename in files:
            filepath = os.path.join(root, filename)
            if os.path.islink(filepath):
                target = os.readlink(filepath)
                if not os.path.isabs(target):
                    target = os.path.join(os.path.dirname(filepath), target)
                if not os.path.exists(target):
                    broken_links.append(filepath)
    return broken_links

def remove_broken_symlinks(broken_links, dry_run=False):
    if not broken_links:
        print("No broken symbolic links found.")
        return

    for link in broken_links:
        if dry_run:
            print(f"[Dry Run] Would remove: {link}")
        else:
            try:
                os.remove(link)
                print(f"Removed: {link}")
            except OSError as e:
                print(f"Failed to remove: {link} ({e})")

def is_regular_file(filename):
    return not filename.endswith("~") and not filename.startswith(".")


def identify_device():
    try:
        with open("/proc/cpuinfo") as f:
            cpuinfo = f.read()
        if "Raspberry Pi" in cpuinfo:
            return "Raspberry Pi"
        if osp.exists("/proc/device-tree/model"):
            with open("/proc/device-tree/model") as f:
                model = f.read().strip().replace("\x00", "")
            if "Radxa" in model or "ROCK Pi" in model\
               or model == "Khadas VIM4"\
               or model == "NVIDIA Jetson Xavier NX Developer Kit":
                return model
        if osp.exists('/usr/local/m5stack/block-mount.sh'):
            return 'm5stack-LLM'
        return "Unknown Device"
    except FileNotFoundError:
        return "Unknown Device"


def create_symlinks(source_dir, target_dir, username=None, dry_run=False):
    added_symlinks = []
    for item in os.listdir(source_dir):
        source_path = os.path.join(source_dir, item)
        if not (osp.isfile(source_path) and is_regular_file(item)):
            continue
        if username is None:
            target_path = os.path.join(target_dir, item)
        else:
            target_path = os.path.join(target_dir, item.replace(".service", f"@{username}.service"))
        if os.path.isfile(source_path):
            if dry_run:
                print(f"Dry-run: Would create symlink: {target_path} -> {source_path}")
            else:
                if os.path.lexists(target_path):
                    os.remove(target_path)
                if username is None:
                    os.symlink(osp.abspath(source_path), osp.abspath(target_path))
                else:
                    # need to copy
                    subprocess.run(["cp", "-f", source_path, target_path])
                print(f"Created symlink: {target_path} -> {source_path}")
                added_symlinks.append(target_path)
    return added_symlinks


def copy_files(source_dir, target_dir, dry_run=False):
    for item in os.listdir(source_dir):
        if not is_regular_file(item):
            continue
        source_path = os.path.join(source_dir, item)
        if os.path.isfile(source_path):
            if dry_run:
                print(f"Dry-run: Would copy {source_path} to {target_dir}")
            else:
                subprocess.run(["cp", "-f", source_path, target_dir])
                print(f"Copied {item} to {target_dir}")


def enable_systemd_services(symlinks, dry_run=False):
    for symlink_path in symlinks:
        item = os.path.basename(symlink_path)
        if item.endswith(".service"):
            if dry_run:
                print(f"Dry-run: Would enable systemd service: {item}")
            else:
                subprocess.run(["systemctl", "enable", item])
                print(f"Enabled systemd service: {item}")


def execute_dtc_command(dry_run, output_path, source_dts):
    if dry_run:
        print(
            f"Dry-run: Would execute 'dtc' command to create device tree blob at '{output_path}' from '{source_dts}'."
        )
    else:
        dtc_command = [
            "dtc",
            "-@",
            "-I",
            "dts",
            "-O",
            "dtb",
            "-o",
            output_path,
            source_dts,
        ]
        subprocess.run(dtc_command)
        print(
            f"Executed 'dtc' command to create device tree blob at '{output_path}' from '{source_dts}'."
        )


def ros_exists():
    ros_types = ['one', 'noetic']
    for ros_type in ros_types:
        if osp.isfile(f"/opt/ros/{ros_type}/setup.bash"):
            return True
    return False


def install_apt_packages(packages, dry_run=False):
    if not packages:
        print("No APT packages specified for installation.")
        return

    if dry_run:
        print("[Dry Run] Would update package lists using 'apt update'.")
        print(f"[Dry Run] Would install APT packages: {', '.join(packages)}")
    else:
        try:
            print("Updating package lists (apt update)...")
            subprocess.run(["apt", "update"], check=True)

            print(f"Installing APT packages: {', '.join(packages)}...")
            install_command = ["apt", "install", "-y"] + packages
            subprocess.run(install_command, check=True)
            print("APT packages installed successfully.")
        except subprocess.CalledProcessError as e:
            print(f"Failed to execute apt command: {e}")
            print("Please check your internet connection and permissions.")
        except FileNotFoundError:
            print("Error: 'apt' command not found. This script assumes a Debian-based system (like Ubuntu, Raspberry Pi OS).")


def main(dry_run=False, enable_oneshot=False):
    if dry_run is False and os.geteuid() != 0:
        print("This script must be run as root.")
        sys.exit(1)

    username = os.getenv("SUDO_USER") or getpass.getuser()

    bin_source_dir = "./bin"
    systemd_source_dir = "./systemd"
    user_systemd_source_dir = "./systemd/user"
    bin_target_dir = "/usr/local/bin"
    systemd_target_dir = "/etc/systemd/system"

    create_symlinks(bin_source_dir, bin_target_dir, dry_run=dry_run)

    if dry_run is False:
        os.makedirs("/usr/local/dict", 0o777, exist_ok=True)
    create_symlinks("./dict", "/usr/local/dict", dry_run=dry_run)

    if dry_run is False:
        os.makedirs("/etc/opt/riberry", 0o777, exist_ok=True)
    create_symlinks("./etc/opt/riberry", "/etc/opt/riberry", dry_run=dry_run)

    added_user_symlinks = []
    added_symlinks = []
    if ros_exists():
        added_symlinks += create_symlinks(
            "./ros/riberry_startup/systemd", systemd_target_dir, dry_run=dry_run
        )
        added_user_symlinks += create_symlinks(
            "./ros/riberry_startup/systemd/user", systemd_target_dir, username=username,
            dry_run=dry_run
        )
    added_symlinks += create_symlinks(
        systemd_source_dir, systemd_target_dir, dry_run=dry_run
    )
    added_user_symlinks += create_symlinks(
        user_systemd_source_dir, systemd_target_dir, username=username,
        dry_run=dry_run
    )
    if enable_oneshot:
        added_symlinks += create_symlinks(
            "./systemd/oneshot", "/etc/systemd/system", dry_run=dry_run
        )

    if identify_device() == "Radxa Zero":
        copy_files("./boot", "/boot", dry_run=dry_run)
        create_symlinks('./bin/radxa-zero', bin_target_dir, dry_run=dry_run)
        execute_dtc_command(
            dry_run,
            "/boot/dtbs/5.10.69-12-amlogic-g98700611d064/amlogic/overlay/meson-g12a-i2c-ee-m1-gpioh-6-gpioh-7.dtbo",
            "./overlays/i2c1.dts",
        )
        execute_dtc_command(
            dry_run,
            "/boot/dtbs/5.10.69-12-amlogic-g98700611d064/amlogic/overlay/meson-g12a-i2c-ee-m3-gpioa-14-gpioa-15.dtbo",
            "./overlays/i2c3.dts",
        )
        execute_dtc_command(
            dry_run,
            "/boot/dtbs/5.10.69-12-amlogic-g98700611d064/amlogic/overlay/meson-g12a-gpio-line-names.dtbo",
            "./overlays/meson-g12a-gpio-line-names.dts",
        )
        added_symlinks += create_symlinks(
            "./systemd/radxa-zero", "/etc/systemd/system", dry_run=dry_run
        )
    packages_to_install = [
        'wireless-tools',  # for iwgetid command
    ]
    if packages_to_install:
        install_apt_packages(packages_to_install, dry_run=dry_run)

    enable_systemd_services(added_symlinks, dry_run=dry_run)
    enable_systemd_services(added_user_symlinks, dry_run=dry_run)

    remove_broken_symlinks(find_broken_symlinks("/etc/systemd/system"),
                           dry_run=dry_run)

    if dry_run:
        print("Dry-run mode: No changes were made.")
    else:
        print("Installation completed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Install script with dry-run option.")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run the script in dry-run mode to display the operations without making any changes.",
    )
    parser.add_argument(
        "--enable-oneshot",
        action="store_true",
        help="Enable the oneshot services like a resize-helper and change-hostname-helper.",
    )

    args = parser.parse_args()
    main(args.dry_run, args.enable_oneshot)
