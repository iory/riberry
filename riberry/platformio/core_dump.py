import re
import struct
import subprocess
import tempfile
import time

from riberry.git_utils import is_valid_github_url

GITHUB_ACTION_ANALYSIS_MESSAGE = "Automatically analyze core dumped cause by Github Action"
URL_PROMPT_MESSAGE = "Maybe you can find the core dump cause in the following URL:"


def _download_elf_file(temp_file, device_name, riberry_firmware_version, lcd_rotation, use_grove):
    import riberry
    from riberry.firmware_update import download_firmware_from_github
    url = f'https://github.com/iory/riberry/releases/download/v{riberry.__version__}-{riberry_firmware_version}/{device_name}-lcd{lcd_rotation}-grove{use_grove}.elf'
    print(f"Downloading firmware elf from {url} to temporary file {temp_file.name}...")
    try:
        download_firmware_from_github(url, temp_file)
        return True
    except Exception as e:
        print(f"Failed to download firmware: {e}")

def download_elf_file(temp_file, com, riberry_firmware_version, lcd_rotation, use_grove):
    from riberry.platformio.device_to_mcu import get_device_to_mcu
    device_name = get_device_to_mcu(com.device_type)
    return _download_elf_file(temp_file, device_name, riberry_firmware_version, lcd_rotation, use_grove)


def format_core_dump(data, com, elf_path=None,
                        riberry_firmware_version=None, lcd_rotation=None, use_grove=None):
    core_dumped = struct.unpack('<I', data[0:4])[0]
    pc = struct.unpack('<I', data[4:8])[0]
    ps = struct.unpack('<I', data[8:12])[0]
    a_regs = struct.unpack('<16I', data[12:76])
    sar = struct.unpack('<I', data[76:80])[0]
    exccause = struct.unpack('<I', data[80:84])[0]
    excvaddr = struct.unpack('<I', data[84:88])[0]
    lbeg = struct.unpack('<I', data[88:92])[0]

    if core_dumped == 0:
        return "No core dump available", False
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

    if elf_path is None:
        temp_file = tempfile.NamedTemporaryFile(suffix=".elf", delete=True)
        ret = download_elf_file(temp_file, com, riberry_firmware_version, lcd_rotation, use_grove)
        if ret:
            elf_path = temp_file.name

    if elf_path is not None:
        output += '\n'
        output += run_addr2line(str(elf_path), pc)
    return output, True


def read_core_dump(com, elf_path=None, retry_count=5,
                   repo_owner="iory", repo_name="riberry"):
    riberry_firmware_version = None
    lcd_rotation = None
    use_grove = None
    for _ in range(retry_count):
        with com.lock_context():
            com.write([0xFD])
            time.sleep(0.01)
            version = com.read().decode().split('_')
            if len(version) >= 3:
                riberry_firmware_version = version[0]
                lcd_rotation = version[1]
                use_grove = version[2]
                break
        time.sleep(0.1)
    formatted_output = ""
    formatted_output += f"Core dumped Firmware version: {riberry_firmware_version}\n"
    formatted_output += f"LCD rotation: {lcd_rotation}\n"
    formatted_output += f"Use Grove: {use_grove}\n"
    formatted_output += f"Device: {com.device_type}\n"
    formatted_output += f"Communication: {com.__class__.__name__}\n\n"

    core_dumped = False
    for _ in range(retry_count):
        success = False
        with com.lock_context():
            com.write([0xFC])
            time.sleep(0.01)
            response = com.read()
            if len(response) > 0:
                try:
                    core_dumped_message, core_dumped = format_core_dump(
                        response,
                        com,
                        elf_path=elf_path,
                        riberry_firmware_version=riberry_firmware_version,
                        lcd_rotation=lcd_rotation,
                        use_grove=use_grove)
                    formatted_output += core_dumped_message
                    success = True
                except Exception as e:
                    print(f"Failed to format core dump: {e}")
        if success:
            break
        time.sleep(0.1)
    print(formatted_output)
    if success and core_dumped:
        from riberry.git_utils import generate_github_issue_url

        # Generate GitHub issue URL
        riberry_firmware_version = riberry_firmware_version or "unknown"
        issue_title = f"Core Dump Report - Firmware {riberry_firmware_version}"
        message = "I have encountered a core dump while running the firmware."
        issue_body = f"{message}\n```\n{formatted_output}\n```"
        issue_url = generate_github_issue_url(repo_owner, repo_name, issue_title, issue_body)

        clickable_url = f"\033]8;;{issue_url}\033\\Click here to create a GitHub issue\033]8;;\033\\"
        print(clickable_url)
        print(f"If not clickable, use this URL:\n{issue_url}")

def generate_github_source_url(addr2line_output, riberry_firmware_version, repo_owner="iory", repo_name="riberry"):
    pattern = r"at (.+):(\d+)$"
    match = re.search(pattern, addr2line_output)
    if not match:
        print("Could not parse addr2line output for file path and line number")
        return None
    file_path = match.group(1)
    line_number = match.group(2)
    repo_root_pattern = rf'/tmp/{repo_name}/'
    repo_root_match = re.search(repo_root_pattern, file_path)
    if not repo_root_match:
        print("Could not find repository root in file path")
        return None
    relative_path = file_path[repo_root_match.end():]
    base_url = f"https://github.com/{repo_owner}/{repo_name}/blob/{riberry_firmware_version}/{relative_path}#L{line_number}"
    if is_valid_github_url(base_url):
        return base_url


def extract_core_dump_info_from_text(text):
    patterns = {
        'riberry_firmware_version': r"Core dumped Firmware version: ([0-9a-f]{7})",  # SHAハッシュ用に修正
        'lcd_rotation': r"LCD rotation: (\d+)",
        'use_grove': r"Use Grove: (\w+)",
        'device_type': r"Device: (.+?)\n",
        'pc_value': r"PC\s*:\s*0x([0-9a-fA-F]{8})"
    }
    extracted_info = {}
    for key, pattern in patterns.items():
        match = re.search(pattern, text)
        if match:
            extracted_info[key] = match.group(1)
        else:
            print(f"Warning: Could not extract {key} from text")
            extracted_info[key] = None
    return extracted_info

def extract_device_info(text):
    try:
        info = extract_core_dump_info_from_text(text)
        if not info:
            raise ValueError("Failed to extract core dump info from text")
        return info
    except Exception as e:
        print(f"Error extracting device info: {e}")
        raise


def run_addr2line(elf_path, pc):
    from riberry.platformio.toolchain import ensure_toolchain
    from riberry.platformio.toolchain import get_addr2line_path
    ensure_toolchain()
    addr2line_path = get_addr2line_path()

    addr2line_cmd = [str(addr2line_path), "-e", elf_path, '-a', '-pfiaC', hex(pc)]
    try:
        output = subprocess.check_output(addr2line_cmd, stderr=subprocess.DEVNULL).decode()
        return f"Executing addr2line command:\n{' '.join(addr2line_cmd)}\n{output}"
    except subprocess.CalledProcessError as e:
        print(f"Error executing addr2line: {e}")
        return ""
    except FileNotFoundError:
        print("Error: addr2line tool not found in PATH")
        return ""

def analyze_core_dump(text):
    try:
        info = extract_device_info(text)
    except ValueError as e:
        print(f"Analysis aborted: {e}")
        return

    # Get device-specific details
    from riberry.platformio.device_to_mcu import get_device_to_mcu
    device_name = get_device_to_mcu(info['device_type'])
    firmware_version = info['riberry_firmware_version']
    lcd_rotation = info['lcd_rotation']
    use_grove = info['use_grove']

    try:
        pc = int(info['pc_value'], 16)
    except (ValueError, KeyError) as e:
        print(f"Error parsing PC value: {e}")
        return

    # Download ELF file
    elf_temp_file = tempfile.NamedTemporaryFile(suffix=".elf", delete=True)
    found_elf_path = _download_elf_file(elf_temp_file, device_name, firmware_version, lcd_rotation, use_grove)
    if not found_elf_path:
        print("Analysis aborted due to ELF download failure")
        return
    elf_path = elf_temp_file.name

    # Run addr2line and get output
    print(GITHUB_ACTION_ANALYSIS_MESSAGE)
    addr2line_output = run_addr2line(elf_path, pc)
    if not addr2line_output:
        print("Failed to get addr2line output")
    else:
        print(addr2line_output)

        # Generate and validate GitHub URL
        url = generate_github_source_url(addr2line_output, firmware_version)
        if url:
            print(URL_PROMPT_MESSAGE)
            print(url)
        else:
            print("Could not generate a valid GitHub URL")
