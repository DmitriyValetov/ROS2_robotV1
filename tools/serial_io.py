"""Работа с последовательным портом напрямую через termios.

Зачем не pyserial: на macOS открытие порта через pyserial дёргает линии
DTR/RTS (ioctl TIOCMBIS/TIOCMBIC), что **перезагружает ESP32** в момент
открытия. Из-за этого теряется начало вывода скетча: attach происходит позже,
чем setup() успевает напечатать свои строки.

Здесь порт открывается без изменения управляющих линий, поэтому плата
продолжает работать и мы читаем её вывод с текущего момента.
"""

import os
import select
import termios
import time

# Таблица скоростей: на macOS termios-константы уже содержат скорость в старших
# битах, Linux — отдельные Bxxx. Обрабатываем оба варианта.
_BAUD = {}
for _name in ("B0", "B50", "B75", "B110", "B134", "B150", "B200", "B300", "B600", "B1200",
              "B1800", "B2400", "B4800", "B9600", "B19200", "B38400", "B57600", "B115200",
              "B230400", "B460800", "B500000", "B576000", "B921600", "B1000000", "B2000000"):
    if hasattr(termios, _name):
        _BAUD[int(_name[1:])] = getattr(termios, _name)


class SerialError(Exception):
    pass


class SerialPort:
    """Минимальная замена pyserial: read/write без сброса платы при открытии."""

    def __init__(self, device, baudrate=115200, timeout=0.3, write=True):
        self.device = device
        self.baudrate = baudrate
        self.timeout = timeout
        self._buf = b""
        flags = os.O_RDWR if write else os.O_RDONLY
        try:
            self.fd = os.open(device, flags | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            if e.errno == 16:
                raise SerialError(f"порт занят другой программой ({e.strerror})") from e
            raise SerialError(f"не открыть {device}: {e.strerror}") from e

        try:
            attrs = termios.tcgetattr(self.fd)
        except termios.error as e:
            os.close(self.fd)
            raise SerialError(f"не настроить {device}: {e}") from e

        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
        iflag = 0
        oflag = 0
        lflag = 0
        cflag = termios.CS8 | termios.CREAD | termios.CLOCAL
        cc = list(cc)
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        speed = _BAUD.get(baudrate)
        if speed is None:
            raise SerialError(f"скорость {baudrate} не поддерживается")
        # TCSANOW: применяем сразу, линии DTR/RTS не трогаем — плата не сбрасывается
        termios.tcsetattr(self.fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, speed, speed, cc])
        termios.tcflush(self.fd, termios.TCIFLUSH)

    # --- pyserial-совместимый интерфейс ---
    @property
    def is_open(self):
        return self.fd >= 0

    @property
    def in_waiting(self):
        try:
            return len(os.read(self.fd, 65536) or b"") + len(self._buf)
        except BlockingIOError:
            return len(self._buf)

    def read(self, size=1):
        """Возвращает до size байт; пустая строка — таймаут (как в pyserial)."""
        if self._buf:
            chunk, self._buf = self._buf[:size], self._buf[size:]
            return chunk
        ready, _, _ = select.select([self.fd], [], [], self.timeout)
        if not ready:
            return b""
        try:
            return os.read(self.fd, size)
        except BlockingIOError:
            return b""

    def readline(self):
        """Читает до \n. Возвращает b"" при таймауте, если ничего не накопилось."""
        deadline = time.monotonic() + self.timeout
        while b"\n" not in self._buf:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            ready, _, _ = select.select([self.fd], [], [], remaining)
            if not ready:
                break
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                continue
            if not chunk:
                break
            self._buf += chunk
        if b"\n" in self._buf:
            line, self._buf = self._buf.split(b"\n", 1)
            return line + b"\n"
        if self._buf:
            line, self._buf = self._buf, b""
            return line
        return b""

    def write(self, data):
        if isinstance(data, str):
            data = data.encode("utf-8")
        return os.write(self.fd, data)

    def flush(self):
        try:
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except termios.error:
            pass

    def close(self):
        if getattr(self, "fd", -1) >= 0:
            try:
                os.close(self.fd)
            finally:
                self.fd = -1

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
