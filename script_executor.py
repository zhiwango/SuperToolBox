def build_command(script_info):
    path = script_info["path"]
    sudo = script_info.get("sudo", False)
    return f"sudo -S {path}" if sudo else path