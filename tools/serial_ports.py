"""Выбор serial-порта: один код для отладочного прогона и для тюнера PID.

Проблема, которую решает модуль: в системе есть Bluetooth-устройства
(/dev/cu.JBLTune720BT, /dev/cu.Buds3), которые выглядят как обычные
callout-порты и перехватывают автопоиск. Плата всегда USB-порт, поэтому
USB-кандидаты имеют приоритет, а известные Bluetooth-порты исключаются.
"""

import os
import re
import sys

try:
    from serial.tools import list_ports
except ImportError:  # pyserial не установлен
    list_ports = None

# Порты, которые точно не плата
EXCLUDE_RE = re.compile(
    r"Bluetooth|debug-console|Buds|JBL|AirPods|WH-1000|SoundCore|Incoming-Port",
    re.IGNORECASE,
)


def _all_ports():
    """Все последовательные порты: описание из pyserial + сырые /dev/cu.* на macOS."""
    ports = []
    if list_ports is not None:
        for p in list_ports.comports():
            ports.append({"device": p.device, "description": p.description or "", "hwid": p.hwid or ""})
    if sys.platform == "darwin":
        import glob

        known = {p["device"] for p in ports}
        for dev in sorted(glob.glob("/dev/cu.*")):
            if dev not in known:
                ports.append({"device": dev, "description": "", "hwid": ""})
    return ports


def is_plausible_board(port):
    """Плата — это USB-порт: либо pyserial видит USB в описании, либо это ttyUSB/ACM."""
    dev = port["device"]
    if EXCLUDE_RE.search(dev) or EXCLUDE_RE.search(port["description"]):
        return False
    if dev.startswith(("/dev/ttyUSB", "/dev/ttyACM")):
        return True
    text = f"{port['description']} {port['hwid']}"
    if re.search(r"USB|UART|CP210|CH34|FTDI|Silicon Labs|wch", text, re.IGNORECASE):
        return True
    return False


def pick_port(explicit=None):
    """Порт платы: явный → из окружения DSH_SERIAL_PORT → первый USB-кандидат."""
    if explicit:
        return explicit
    env = os.environ.get("DSH_SERIAL_PORT")
    if env:
        return env
    candidates = [p["device"] for p in _all_ports() if is_plausible_board(p)]
    return candidates[0] if candidates else None


def describe_ports():
    """Человекочитаемый список портов с пометкой, какие годятся."""
    rows = []
    for p in _all_ports():
        rows.append((p["device"], p["description"] or "без описания", is_plausible_board(p)))
    return rows
