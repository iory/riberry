def get_device_to_mcu(device_model):
    if device_model == 'm5stack-LLM':
        device_name = 'm5stack-basic'
    elif "Radxa" in device_model or "ROCK Pi" in device_model \
        or device_model == "Khadas VIM4" \
        or device_model == "NVIDIA Jetson Xavier NX Developer Kit":
        device_name = 'm5stack-atoms3'
    else:
        raise NotImplementedError(f"Not supported device {device_model}. Please feel free to add the device name to the list or ask the developer to add it.")
    return device_name
