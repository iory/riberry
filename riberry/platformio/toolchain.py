import os
import subprocess


def get_addr2line_path():
    toolchain_dir = os.path.expanduser("~/.platformio/packages/toolchain-xtensa-esp32s3")
    addr2line_path = os.path.join(toolchain_dir, "bin", "xtensa-esp32s3-elf-addr2line")
    return addr2line_path


def ensure_toolchain():
    addr2line_path = get_addr2line_path()
    if not os.path.exists(addr2line_path):
        print(f"Toolchain not found at {addr2line_path}. Installing espressif32 platform...")
        try:
            subprocess.run(["pio", "pkg", "install", "-g", "-t", "toolchain-xtensa-esp32s3"], check=True)
            print("Espressif32 platform (including toolchain) installed successfully.")
        except subprocess.CalledProcessError as e:
            print(f"Failed to install platform: {e}")
            return False
        if not os.path.exists(addr2line_path):
            print("Toolchain still not found after installation. Check PlatformIO setup.")
            return False
    return True
