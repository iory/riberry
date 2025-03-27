import os
import subprocess

env = DefaultEnvironment()  # NOQA
print(f"BUILD_DIR: {env.subst('$BUILD_DIR')}")
print(f"PROGNAME: {env.subst('$PROGNAME')}")
firmware_path = os.path.join(env.subst("$BUILD_DIR"), env.subst("$PROGNAME") + ".bin")
print(f"firmware_path: {firmware_path}")
upload_port = env.subst("$UPLOAD_PORT")
remote_firmware = "firmware.bin"
ssh_port = "22"
upload_flags = env.get("UPLOAD_FLAGS", [])
print(f"upload_port: {upload_port}")
print(f"upload_flags: {upload_flags}")

def is_scp_upload():
    return "scp" in upload_port.lower()

if is_scp_upload():
    scp_host = upload_port.replace("scp:", "")
    temp_remote_firmware = f"/tmp/temp_firmware_{os.path.basename(firmware_path)}"
    scp_command = ["scp", "-P", ssh_port, firmware_path, f"{scp_host}:{temp_remote_firmware}"]
    ssh_command = [
        "ssh", "-p", ssh_port, scp_host,
        f"PATH=$HOME/.local/bin:$PATH riberry-update-firmware -f {temp_remote_firmware} && rm -f {temp_remote_firmware}"
    ]

    try:
        print(f"Transferring {firmware_path} to {scp_host}:{temp_remote_firmware} via SCP...")
        subprocess.check_call(scp_command)
        print(f"Executing firmware update on {scp_host} and cleaning up...")
        subprocess.check_call(ssh_command)
        print("Firmware update via SCP completed successfully and temporary file removed!")
    except subprocess.CalledProcessError as e:
        print(f"SCP/SSH Error: {e}")
        try:
            subprocess.check_call(["ssh", "-p", ssh_port, scp_host, f"rm -f {temp_remote_firmware}"])
            print("Cleaned up temporary file despite error")
        except subprocess.CalledProcessError:
            print("Failed to clean up temporary file")
        raise
else:
    print("Skipping custom upload; using standard USB upload.")
