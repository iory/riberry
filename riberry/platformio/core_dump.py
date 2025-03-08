import struct
import subprocess
import tempfile
import time


def format_core_dump(data, elf_path=None):
    core_dumped = struct.unpack('<I', data[0:4])[0]
    pc = struct.unpack('<I', data[4:8])[0]
    ps = struct.unpack('<I', data[8:12])[0]
    a_regs = struct.unpack('<16I', data[12:76])
    sar = struct.unpack('<I', data[76:80])[0]
    exccause = struct.unpack('<I', data[80:84])[0]
    excvaddr = struct.unpack('<I', data[84:88])[0]
    lbeg = struct.unpack('<I', data[88:92])[0]

    if core_dumped == 0:
        return "No core dump available"
    output = (
        f"PC      : 0x{pc:08x}  PS      : 0x{ps:08x}  "
        + f"A0      : 0x{a_regs[0]:08x}  A1      : 0x{a_regs[1]:08x}\n"
        + f"A2      : 0x{a_regs[2]:08x}  A3      : 0x{a_regs[3]:08x}  "
        + f"A4      : 0x{a_regs[4]:08x}  A5      : 0x{a_regs[5]:08x}\n"
        + f"A6      : 0x{a_regs[6]:08x}  A7      : 0x{a_regs[7]:08x}  "
        + f"A8      : 0x{a_regs[8]:08x}  A9      : 0x{a_regs[9]:08x}\n"
        + f"A10     : 0x{a_regs[10]:08x}  A11     : 0x{a_regs[11]:08x}  "
        + f"A12     : 0x{a_regs[12]:08x}  A13     : 0x{a_regs[13]:08x}\n"
        + f"A14     : 0x{a_regs[14]:08x}  A15     : 0x{a_regs[15]:08x}  "
        + f"SAR     : 0x{sar:08x}  EXCCAUSE: 0x{exccause:08x}\n"
        + f"EXCVADDR: 0x{excvaddr:08x}  LBEG    : 0x{lbeg:08x}  "
        + '\n'
    )

    if elf_path is not None:
        from riberry.platformio.toolchain import ensure_toolchain
        from riberry.platformio.toolchain import get_addr2line_path
        ensure_toolchain()
        addr2line_path = get_addr2line_path()
        addr2line_cmd = [str(addr2line_path), "-e", str(elf_path), '-a', '-pfiaC', str(hex(pc))]
        output += '\n'
        output += 'Executing addr2line command:\n'
        output += ' '.join(addr2line_cmd) + '\n'
        output += subprocess.check_output(
            addr2line_cmd, stderr=subprocess.DEVNULL
        ).decode()
    return output


def read_core_dump(com, elf_path=None, retry_count=5,
                   repo_owner="iory", repo_name="riberry"):
    formatted_output = ""
    for _ in range(retry_count):
        with com.lock_context():
            com.write([0xFD])
            time.sleep(0.01)
            version = com.read().decode().split('_')
            if len(version) >= 3:
                import riberry
                from riberry.firmware_update import download_firmware_from_github
                riberry_git_version, lcd_rotation, use_grove = version[:3]
                formatted_output += f"Core dumped Firmware version: {riberry_git_version}\n"
                formatted_output += f"LCD rotation: {lcd_rotation}\n"
                formatted_output += f"Use Grove: {use_grove}\n"

                if elf_path is None:
                    model = com.device_type
                    if model == 'm5stack-LLM':
                        device_name = 'm5stack-basic'
                    elif "Radxa" in model or "ROCK Pi" in model \
                        or model == "Khadas VIM4" \
                        or model == "NVIDIA Jetson Xavier NX Developer Kit":
                        device_name = 'm5stack-atoms3'
                    else:
                        raise NotImplementedError(f"Not supported device {model}. Please feel free to add the device name to the list or ask the developer to add it.")

                    url = f'https://github.com/iory/riberry/releases/download/v{riberry.__version__}-{riberry_git_version}/{device_name}-lcd{lcd_rotation}-grove{use_grove}.elf'
                    temp_file = tempfile.NamedTemporaryFile(suffix=".elf", delete=True)
                    print(f"Downloading firmware elf from {url} to temporary file {temp_file.name}...")
                    try:
                        elf_path = download_firmware_from_github(url, temp_file)
                    except Exception as e:
                        print(f"Failed to download firmware: {e}")
                break
        time.sleep(0.1)
    if formatted_output == "":
        formatted_output += "Failed to read core dump version\n"
    formatted_output += f"Device: {com.device_type}\n"
    formatted_output += f"Communication: {com.__class__.__name__}\n\n"

    for _ in range(retry_count):
        success = False
        with com.lock_context():
            com.write([0xFC])
            time.sleep(0.01)
            response = com.read()
            if len(response) > 0:
                try:
                    formatted_output += format_core_dump(response,
                                                         elf_path=elf_path)
                    success = True
                except Exception as e:
                    print(f"Failed to format core dump: {e}")
        if success:
            break
        time.sleep(0.1)
    if success:
        from riberry.git_utils import generate_github_issue_url
        print(formatted_output)

        # Generate GitHub issue URL
        issue_title = f"Core Dump Report - Firmware {version}"
        message = "I have encountered a core dump while running the firmware."
        issue_body = f"{message}\n```\n{formatted_output}\n```"
        issue_url = generate_github_issue_url(repo_owner, repo_name, issue_title, issue_body)

        clickable_url = f"\033]8;;{issue_url}\033\\Click here to create a GitHub issue\033]8;;\033\\"
        print(clickable_url)
        print(f"If not clickable, use this URL:\n{issue_url}")
