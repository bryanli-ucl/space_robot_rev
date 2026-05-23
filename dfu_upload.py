Import("env")

import os
import subprocess
import time

import serial
import serial.tools.list_ports


def find_serial_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = (port.description or "").lower()
        hwid = (port.hwid or "").lower()
        if (
            "giga" in desc
            or "arduino" in desc
            or "stm32" in desc
            or "vid:pid=2341" in hwid
        ):
            return port.device
    return None


def touch_1200bps(port):
    print(f"[AUTO-DFU] Touching {port} at 1200bps")
    try:
        ser = serial.Serial(port, 1200)
        time.sleep(0.2)
        ser.close()
    except Exception as exc:
        print(f"[AUTO-DFU] Touch failed: {exc}")


def wait_for_dfu():
    print("[AUTO-DFU] Waiting for DFU device...")
    for _ in range(30):
        result = subprocess.run(["dfu-util", "-l"], capture_output=True, text=True)
        if "Found DFU" in result.stdout:
            print("[AUTO-DFU] DFU device found")
            return True
        time.sleep(1)
    return False


def upload_bin(source, target, env):
    firmware_path = os.path.join(env.subst("$BUILD_DIR"), "firmware.bin")
    offset = env.BoardConfig().get("upload.offset_address", "0x08040000")

    port = find_serial_port()
    if not port:
        print("[AUTO-DFU] No serial port found")
        return

    touch_1200bps(port)
    if not wait_for_dfu():
        print("[AUTO-DFU] DFU device not detected")
        return

    cmd = ["dfu-util", "-a", "0", "-D", firmware_path, "-s", f"{offset}:leave"]
    print(f"[AUTO-DFU] Uploading firmware to {offset}")
    print(" ".join(cmd))

    result = subprocess.run(cmd)
    if result.returncode != 0:
        print("[AUTO-DFU] Upload failed")
    else:
        print("[AUTO-DFU] Upload success")


env.Replace(UPLOADER="python")
env.Replace(UPLOADCMD=upload_bin)

env.AddCustomTarget(
    name="uploadbin",
    dependencies=None,
    actions=[upload_bin],
    title="Upload BIN via DFU",
    description="Auto DFU upload",
)
