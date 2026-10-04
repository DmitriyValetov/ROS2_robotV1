#!/usr/bin/env python3
"""Отладочный прогон прошивки ESP32: залить скетч и собрать его вывод.

Одна команда делает цикл «прошить → прочитать serial» и печатает результат,
поэтому подходит для итеративной отладки без GUI.

Примеры:

    # прошить скетч и 10 секунд читать вывод
    tools/serial_debug.py --sketch code/motors/who_am_i.ino

    # только послушать порт (ничего не прошивать)
    tools/serial_debug.py --record 10

    # прошить и ждать нужную строку до 30 секунд
    tools/serial_debug.py --sketch code/motors/test_imu_mpu6500.ino --wait-for "Готово"

    # прошить свой скетч с пином светодиода и прочитать 5 секунд
    tools/serial_debug.py --sketch code/motors/exp5_minor_course_fixes.ino --seconds 5
"""

import argparse
import os
import re
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from serial_ports import pick_port, describe_ports  # noqa: E402
from serial_io import SerialPort, SerialError  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_FQBN = "esp32:esp32:esp32"


def port_busy(port):
    """Кто держит порт: возвращает описание процесса или None."""
    try:
        out = subprocess.run(["lsof", port], capture_output=True, text=True, timeout=10).stdout
    except Exception:
        return None
    lines = [l for l in out.splitlines()[1:] if l.strip()]
    if lines:
        return lines[0].split()[0]
    return None


def upload(sketch, fqbn, port, verbose=False):
    """Прошивает скетч. Arduino требуют, чтобы имя папки совпадало с .ino."""
    sketch = os.path.abspath(sketch)
    if not os.path.isfile(sketch):
        print(f"Скетч не найден: {sketch}", file=sys.stderr)
        return False
    name = os.path.splitext(os.path.basename(sketch))[0]
    workdir = tempfile.mkdtemp(prefix=f"dsh-{name}-")
    sk_dir = os.path.join(workdir, name)
    os.makedirs(sk_dir, exist_ok=True)
    with open(sketch, "rb") as src, open(os.path.join(sk_dir, f"{name}.ino"), "wb") as dst:
        dst.write(src.read())
    build = os.path.join(workdir, "build")

    cmd = ["arduino-cli", "compile", "--fqbn", fqbn, "--build-path", build,
           "--upload", "-p", port, sk_dir]
    print(f"→ {name}: компилирую и прошиваю в {port} …")
    res = subprocess.run(cmd, capture_output=True, text=True)
    tail = (res.stdout + res.stderr).strip().splitlines()
    if res.returncode != 0:
        print("ПРОШИВКА НЕ УДАЛАСЬ:")
        print("\n".join(tail[-25:]))
        return False
    summary = [l for l in tail if "памяти устройства" in l or "динамической памяти" in l]
    print("→ ок: " + (summary[0].strip() if summary else "сборка прошла"))
    return True


def record(port, baud, seconds, wait_for=None, echo=True):
    """Читает порт, печатает строки. Возвращает список строк."""
    if port_busy(port):
        print(f"Порт {port} занят процессом «{port_busy(port)}» — закройте монитор порта.", file=sys.stderr)
        return None
    try:
        # write=True: порт нужен и для команд; линии DTR/RTS не трогаем, плата не сбрасывается
        ser = SerialPort(port, baud, timeout=0.3)
    except SerialError as e:
        print(f"Не открыть {port}: {e}", file=sys.stderr)
        return None

    lines = []
    deadline = time.time() + seconds
    pattern = re.compile(wait_for) if wait_for else None
    try:
        while time.time() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").rstrip()
            if not line:
                continue
            lines.append(line)
            if echo:
                print(line)
            if pattern and pattern.search(line):
                print(f"— совпадение с «{wait_for}», дальше не читаю")
                break
    finally:
        ser.close()
    return lines


def main():
    ap = argparse.ArgumentParser(description="Прошить скетч и прочитать serial ESP32")
    ap.add_argument("--sketch", help="путь к .ino (если не указан — только чтение порта)")
    ap.add_argument("--port", help="serial-порт (по умолчанию первый USB)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--fqbn", default=DEFAULT_FQBN, help=f"по умолчанию {DEFAULT_FQBN}")
    ap.add_argument("--seconds", type=float, default=10.0, help="сколько секунд читать вывод")
    ap.add_argument("--wait-for", help="регулярка: остановить чтение при совпадении")
    ap.add_argument("--record", type=float, metavar="СЕК", help="только слушать порт указанное число секунд, не прошивать")
    ap.add_argument("--no-echo", action="store_true", help="не печатать строки (только итог)")
    ap.add_argument("--list-ports", action="store_true", help="показать порты и выйти")
    args = ap.parse_args()

    if args.list_ports:
        print("Порты (★ — годится для платы):")
        for dev, desc, ok in describe_ports():
            print(f"  {'★' if ok else ' '} {dev:<30} {desc}")
        return 0

    port = pick_port(args.port)
    if not port:
        print("USB-порт не найден. Подключите плату или укажите --port.", file=sys.stderr)
        return 2

    busy = port_busy(port)
    if busy:
        print(f"Порт {port} занят процессом «{busy}» — закройте монитор порта Arduino IDE и повторите.",
              file=sys.stderr)
        return 3

    seconds = args.seconds
    if args.record is not None:
        seconds = args.record            # режим «только слушать»
    elif args.sketch:
        if not upload(args.sketch, args.fqbn, port):
            return 1
        time.sleep(1.5)   # ESP32 перезагружается после прошивки, ждём загрузчик

    lines = record(port, args.baud, seconds, args.wait_for, echo=not args.no_echo)
    if lines is None:
        return 3

    print(f"\n=== собрано строк: {len(lines)} за {seconds} с ===")
    for pattern in ("WHO_AM_I", "ID 0x", "Найден", "ИТОГ", "Ошибка", "ERROR", "PWR_MGMT_1", "Accel"):
        hits = [l for l in lines if pattern in l]
        if hits:
            print(f"  [{pattern}] {hits[0][:150]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
