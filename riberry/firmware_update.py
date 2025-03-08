import math
import os
import subprocess
import tempfile
import time

import requests

import riberry
from riberry.platformio.device_to_mcu import get_device_to_mcu


def download_firmware_from_github(url, save_path):
    response = requests.get(url, stream=True)
    response.raise_for_status()
    with open(save_path.name, "wb") as f:
        for chunk in response.iter_content(chunk_size=8192):
            if chunk:
                f.write(chunk)
    return save_path.name


def update_firmware(com, lcd_rotation=1, use_grove=0, firmware_path=None):
    riberry_git_version = None
    if firmware_path is None:
        riberry_git_version = subprocess.check_output(
            ["git", "log", "-1", "--pretty=format:%h"],
            cwd=os.path.dirname(riberry.__file__),
            universal_newlines=True
        ).strip()
        device_name = get_device_to_mcu(com.device_type)
        url = f'https://github.com/iory/riberry/releases/download/v{riberry.__version__}-{riberry_git_version}/{device_name}-lcd{lcd_rotation}-grove{use_grove}.bin'
        temp_file = tempfile.NamedTemporaryFile(suffix=".bin", delete=True)
        print(f"Downloading firmware from {url} to temporary file {temp_file.name}...")
        firmware_path = download_firmware_from_github(url, temp_file)
        print(f"Firmware downloaded to {firmware_path}")

    with open(firmware_path, "rb") as f:
        firmware_data = f.read()

    with com.lock_context():
        com.write([0xFF])
        chunk_size = 512 * 2 * 2
        total = len(firmware_data)
        time.sleep(7)
        print(f"Uploading {total} bytes...")
        com.write([total & 0xFF, (total >> 8) & 0xFF, (total >> 16) & 0xFF, (total >> 24) & 0xFF], raw=True)

        time.sleep(5.0)

        start_time = time.time()
        chunks_total = math.ceil(total / chunk_size)

        for i in range(0, total, chunk_size):
            com.reset_input_buffer()
            elapsed_time = time.time() - start_time

            chunks_processed = (i // chunk_size) + 1
            chunks_remaining = chunks_total - chunks_processed

            estimated_time_remaining = (elapsed_time / chunks_processed) * chunks_remaining
            estimated_completion_time = time.time() + estimated_time_remaining

            print(f"Uploading {i}/{total} bytes... Estimated time remaining: {estimated_time_remaining:.2f} seconds (Completion time: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(estimated_completion_time))})")

            chunk = firmware_data[i:i+chunk_size]
            send_data = list(chunk)
            com.write(send_data, raw=True)
            time.sleep(0.03)
        time.sleep(5.0)

    with com.lock_context():
        com.write([0xFD])
        time.sleep(0.01)
        version = com.read().decode('utf-8')
        print(f"Firmware version: {version}")
        version = version.split('_')
        if len(version) < 3:
            print("Invalid firmware version. Firmware update failed.")
        else:
            if riberry_git_version is not None:
                if version[0] != riberry_git_version:
                    print("Invalid firmware version. Firmware update failed.")
                else:
                    print("Firmware update succeeded.")
