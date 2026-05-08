Import("env")

import serial
import serial.tools.list_ports
import time
import subprocess
import os

#
# Arduino GIGA auto DFU uploader
#

def find_serial_port():
    ports = serial.tools.list_ports.comports()

    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()

        if (
            "giga" in desc
            or "arduino" in desc
            or "stm32" in desc
            or "vid:pid=2341" in hwid
        ):
            return p.device

    return None


def touch_1200bps(port):
    print(f"[AUTO-DFU] Touching {port} at 1200bps")

    try:
        ser = serial.Serial(port, 1200)
        time.sleep(0.2)
        ser.close()
    except Exception as e:
        print(f"[AUTO-DFU] Touch failed: {e}")


def wait_for_dfu():
    print("[AUTO-DFU] Waiting for DFU device...")

    for _ in range(30):

        result = subprocess.run(
            ["dfu-util", "-l"],
            capture_output=True,
            text=True
        )

        if "Found DFU" in result.stdout:
            print("[AUTO-DFU] DFU device found")
            return True

        time.sleep(1)

    return False


def upload_bin(source, target, env):

    firmware_path = os.path.join(
        env.subst("$BUILD_DIR"),
        "firmware.bin"
    )

    port = find_serial_port()

    if not port:
        print("[AUTO-DFU] No serial port found")
        return

    touch_1200bps(port)

    if not wait_for_dfu():
        print("[AUTO-DFU] DFU device not detected")
        return

    cmd = [
        "dfu-util",
        "-a", "0",
        "-D", firmware_path,
        "-s", "0x08040000:leave"
    ]

    print("[AUTO-DFU] Uploading firmware...")
    print(" ".join(cmd))

    result = subprocess.run(cmd)

    if result.returncode != 0:
        print("[AUTO-DFU] Upload failed")
    else:
        print("[AUTO-DFU] Upload success")


env.Replace(
    UPLOADER="python"
)

env.AddCustomTarget(
    name="uploadbin",
    dependencies=None,
    actions=[
        upload_bin
    ],
    title="Upload BIN via DFU",
    description="Auto DFU upload"
)

env.Replace(
    UPLOADCMD=upload_bin
)