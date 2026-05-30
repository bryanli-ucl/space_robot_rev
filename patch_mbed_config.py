Import("env")

from pathlib import Path


if env["PIOENV"] == "giga_r1_m7":
    framework_dir = Path(env.PioPlatform().get_package_dir("framework-arduino-mbed"))
    config_path = framework_dir / "variants" / "GIGA" / "mbed_config.h"
    text = config_path.read_text()
    old = "#define MBED_CONF_LWIP_TCPIP_THREAD_STACKSIZE                                   1200"
    new = "#define MBED_CONF_LWIP_TCPIP_THREAD_STACKSIZE                                   4096"

    if old in text:
        config_path.write_text(text.replace(old, new, 1))
        print("Patched GIGA lwIP tcpip thread stack size to 4096 bytes")
    elif new in text:
        print("GIGA lwIP tcpip thread stack size already patched")
    else:
        print("WARNING: could not find GIGA lwIP tcpip thread stack size setting")
