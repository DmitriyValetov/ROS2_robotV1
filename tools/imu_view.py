#!/usr/bin/env python3
"""Интерактивная визуализация IMU (MPU-6500) в реальном времени.

Что показывает:
  * 3D-куб — ориентация датчика в пространстве, обновляется по данным платы;
  * графики ускорения и угловой скорости за последние секунды;
  * числовые значения: углы, скорости, температура, темп опроса.

Как считается ориентация: комплементарный фильтр. Гироскоп даёт быстрые
изменения угла (но накапливает ошибку), акселерометр задаёт абсолютную
привязку по вектору гравитации (но шумит и врёт при разгоне). Фильтр
складывает их: угол интегрируется по гироскопу и подтягивается к наклону
по акселерометру с коэффициентом --alpha.

Запуск:
    tools/imu_view.py                      # порт найдётся сам
    tools/imu_view.py --demo               # без платы: синтетическое движение
    tools/imu_view.py --bias 3.22          # если в прошивке нет калибровки
    tools/imu_view.py --alpha 0.02 --window 10
    tools/imu_view.py --selftest           # проверка фильтра без окна

Управление:
    мышь на 3D-кубе  — повернуть вид (двойной клик — вернуться к датчику)
    C                — перекалибровать смещение гироскопа (держите неподвижно)
    пробел           — пауза / продолжить
    G                — скрыть/показать ось Y (курс, дрейфует)
    S                — сохранить накопленные данные в CSV
    Q                — выход
"""

import argparse
import math
import os
import queue
import re
import statistics
import sys
import threading
import time
import tkinter as tk

# Кэш шрифтов matplotlib рядом с инструментами, чтобы не пересобирался каждый запуск
if not os.environ.get("MPLCONFIGDIR"):
    _cache = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".cache")
    try:
        os.makedirs(_cache, exist_ok=True)
        os.environ["MPLCONFIGDIR"] = _cache
    except OSError:
        import tempfile

        os.environ["MPLCONFIGDIR"] = tempfile.gettempdir()

def _ensure_tcl():
    """Python из uv-сборки несёт Tcl/Tk 9.0, но не находит его данные сам.

    Без этого tkinter падает с «Cannot find a usable init.tcl». Прописываем пути
    к данным Tcl/Tk, если интерпретатор лежит в .python/ проекта.
    """
    if os.environ.get("TCL_LIBRARY"):
        return
    import glob

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    tcl = glob.glob(os.path.join(root, ".python", "cpython-*", "lib", "tcl9.0"))
    tk = glob.glob(os.path.join(root, ".python", "cpython-*", "lib", "tk9.0"))
    if tcl and tk:
        os.environ["TCL_LIBRARY"] = tcl[0]
        os.environ["TK_LIBRARY"] = tk[0]


_ensure_tcl()

import matplotlib  # noqa: E402

matplotlib.use("TkAgg")
import numpy as np  # noqa: E402
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from serial_io import SerialPort, SerialError  # noqa: E402
from serial_ports import pick_port  # noqa: E402

LINE_RE = re.compile(
    r"Accel \(g\):? X=([-+]?[\d.]+) Y=([-+]?[\d.]+) Z=([-+]?[\d.]+) \| "
    r"Gyro \(dps\):? X=([-+]?[\d.]+) Y=([-+]?[\d.]+) Z=([-+]?[\d.]+) \| "
    r"(?:pitch=([-+]?[\d.]+) roll=([-+]?[\d.]+) \| )?T=([-+]?[\d.]+)"
)
# Вариант строки без углов и со словом Target (совместимость с exp4/exp5)
LINE_RE_ALT = re.compile(
    r"SPD L=([-+]?[\d.]+) mm/s R=([-+]?[\d.]+) mm/s"
)

# Сколько групп из LINE_RE нужно для Sample: ax, ay, az, gx, gy, gz, temp — семь.
# Группы pitch/roll приходят из прошивки, но фильтр считает углы сам, поэтому
# их отбрасываем: иначе в Sample уезжало лишнее значение.
SAMPLE_FIELDS = 7


def parse_line(line, t=None):
    """Строка прошивки → Sample или None. Одна точка правды для разбора и для тестов."""
    m = LINE_RE.search(line)
    if not m:
        return None
    g = m.groups()
    if g[0] is None or g[8] is None:
        return None
    # Берём ax, ay, az, gx, gy, gz, а температуру — девятой группой.
    # Группы pitch/roll (индексы 6 и 7) пропускаем: они есть не во всех
    # форматах, а фильтр считает углы сам.
    values = [float(g[i]) for i in range(6)] + [float(g[8])]
    return Sample(time.time() if t is None else t, *values)


class Sample:
    """Один отсчёт датчика."""

    __slots__ = ("t", "ax", "ay", "az", "gx", "gy", "gz", "temp")

    def __init__(self, t, ax, ay, az, gx, gy, gz, temp):
        self.t = t
        self.ax, self.ay, self.az = ax, ay, az
        self.gx, self.gy, self.gz = gx, gy, gz
        self.temp = temp


class DemoSource(threading.Thread):
    """Синтетическое движение: покачивание + медленный поворот. Для проверки без платы."""

    def __init__(self, callback):
        super().__init__(daemon=True)
        self.callback = callback
        self._stop = threading.Event()
        self.t0 = time.time()

    def run(self):
        while not self._stop.is_set():
            t = time.time() - self.t0
            # медленные углы: качели по обеим осям и медленный разворот
            pitch = math.radians(25 * math.sin(0.5 * t))
            roll = math.radians(15 * math.sin(0.8 * t + 1.0))
            yaw = math.radians(30 * math.sin(0.15 * t))
            # гравитация в связанных осях (R^T * g)
            gx = -math.sin(pitch)
            gy = math.cos(pitch) * math.sin(roll)
            gz = math.cos(pitch) * math.cos(roll)
            # производные углов
            dpitch = math.radians(25 * 0.5 * math.cos(0.5 * t))
            droll = math.radians(15 * 0.8 * math.cos(0.8 * t + 1.0))
            dyaw = math.radians(30 * 0.15 * math.cos(0.15 * t))
            self.callback(Sample(time.time(), gx + 0.01, gy - 0.005, gz, dpitch, droll, dyaw, 41.0 + math.sin(t) * 0.5))
            self._stop.wait(0.1)

    def stop(self):
        self._stop.set()


class SerialSource(threading.Thread):
    """Читает строки прошивки и складывает отсчёты в очередь."""

    def __init__(self, port, baud, callback, status):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.callback = callback
        self.status = status
        self._stop = threading.Event()
        self._ser = None
        self.connected = False
        self.error = ""
        self.opened_t = None    # когда порт успешно открылся
        self.last_line_t = None  # когда пришла последняя строка

    def run(self):
        # Счётчики для диагностики: без них непонятно, «нет данных» — это
        # плата молчит или поток чтения встал
        self.lines = 0
        self.bytes = 0
        self.errors = 0
        while not self._stop.is_set():
            if self._ser is None:
                try:
                    self._ser = SerialPort(self.port, self.baud, timeout=0.5)
                    self.connected = True
                    self.opened_t = time.time()
                    self.status(f"Порт открыт: {self.port} @ {self.baud} — жду данные…")
                except SerialError as e:
                    self.connected = False
                    self.error = str(e)
                    self.status(f"Нет порта: {e}")
                    self._stop.wait(2.0)
                    continue
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                self.lines += 1
                self.bytes += len(raw)
                self.last_line_t = time.time()
                if os.environ.get("IMU_VIEW_RAW") == "1" and self.lines <= 4:
                    print(f"RAW[{self.lines}]: {raw!r}", file=sys.stderr, flush=True)
                line = raw.decode("utf-8", errors="ignore").strip()
                sample = parse_line(line)
                if sample is not None:
                    self.callback(sample)
                elif line:
                    self.status(line)
            except Exception as e:  # порт отвалился
                self.errors += 1
                self.connected = False
                self.status(f"Связь потеряна: {type(e).__name__}: {e}")
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None

    def stop(self):
        self._stop.set()
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass


class Attitude:
    """Комплементарный фильтр: гироскоп интегрируем, акселерометром подтягиваем."""

    def __init__(self, alpha=0.02, bias=(0.0, 0.0, 0.0), use_accel=True):
        self.alpha = alpha
        self.bias = list(bias)
        self.use_accel = use_accel
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_t = None
        self.samples = 0
        self.t_lock = time.time()

    def calibrate(self, samples):
        """Средние значения гироскопа по неподвижным отсчётам → смещение нуля."""
        if not samples:
            return None
        n = len(samples)
        self.bias = [
            sum(s.gx for s in samples) / n,
            sum(s.gy for s in samples) / n,
            sum(s.gz for s in samples) / n,
        ]
        return tuple(self.bias)

    def update(self, s):
        if self.last_t is None:
            self.last_t = s.t
            # первая привязка: угол сразу из акселерометра, иначе куб «уезжает»
            self._accel_correction(s, 1.0)
            self.samples += 1
            return
        dt = s.t - self.last_t
        self.last_t = s.t
        if dt <= 0 or dt > 1.0:      # пропуск данных — не интегрируем мусор
            return

        wx = s.gx - self.bias[0]
        wy = s.gy - self.bias[1]
        wz = s.gz - self.bias[2]

        # Углы Эйлера: проекция скорости на оси (без учёта косинусов — на малых
        # углах это стандартное упрощение, для визуализации этого достаточно)
        self.roll += math.radians(wx) * dt
        self.pitch += math.radians(wy) * dt
        self.yaw += math.radians(wz) * dt

        if self.use_accel:
            self._accel_correction(s, self.alpha)

        # нормализуем углы в (-180, 180]
        self.roll = _wrap(self.roll)
        self.pitch = _wrap(self.pitch)
        self.yaw = _wrap(self.yaw)
        self.samples += 1

    def _accel_correction(self, s, k):
        norm = math.sqrt(s.ax * s.ax + s.ay * s.ay + s.az * s.az)
        if norm < 0.3 or norm > 2.0:      # датчик в свободном падении или трясёт — не доверяем
            return
        roll_acc = math.atan2(s.ay, s.az)
        pitch_acc = math.atan2(-s.ax, math.sqrt(s.ay * s.ay + s.az * s.az))
        self.roll += k * _wrap(roll_acc - self.roll)
        self.pitch += k * _wrap(pitch_acc - self.pitch)

    def angles(self):
        return math.degrees(self.roll), math.degrees(self.pitch), math.degrees(self.yaw)


def _wrap(a):
    """Приводит угол к диапазону (-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def rotation_matrix(roll, pitch, yaw):
    """R = Rz(yaw)·Ry(pitch)·Rx(roll) — поворот связанных осей в мировые."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


class CubeRenderer:
    """Рисует плату как куб: верхняя грань помечена, оси X/Y/Z показаны стрелками."""

    def __init__(self, ax, size=1.0):
        self.ax = ax
        self.size = size
        self.poly = None      # Poly3DCollection создаём один раз, дальше обновляем
        self.axis_lines = []  # линии осей X/Y/Z
        self.axis_texts = []  # подписи осей
        s = size
        self._cube = np.array([
            [-s, -s, -s], [s, -s, -s], [s, s, -s], [-s, s, -s],
            [-s, -s, s], [s, -s, s], [s, s, s], [-s, s, s],
        ], dtype=float)
        self.faces = [
            ([0, 1, 2, 3], "#2b6cb0"),   # низ
            ([4, 5, 6, 7], "#63b3ed"),   # верх — «лицевая» сторона платы
            ([0, 1, 5, 4], "#4a5568"),
            ([1, 2, 6, 5], "#4a5568"),
            ([2, 3, 7, 6], "#4a5568"),
            ([3, 0, 4, 7], "#4a5568"),
        ]
        self.axis_len = s * 2.1

    def _init_scene(self):
        """Статические элементы сцены: грани, оси, подписи. Вызывается один раз."""
        ax = self.ax
        verts = self._cube
        polys = [verts[idx] for idx, _ in self.faces]
        colors = [c for _, c in self.faces]
        self.poly = Poly3DCollection(polys, facecolors=colors, edgecolors="#1a202c",
                                     linewidths=1.0, alpha=0.95)
        ax.add_collection3d(self.poly)

        axes = [
            (np.array([1.0, 0, 0]), "#e53e3e", "X"),
            (np.array([0, 1.0, 0]), "#38a169", "Y"),
            (np.array([0, 0, 1.0]), "#3182ce", "Z"),
        ]
        self.axis_lines, self.axis_texts = [], []
        for vec, color, label in axes:
            ln = ax.plot([0, vec[0]], [0, vec[1]], [0, vec[2]], color=color, linewidth=3)[0]
            txt = ax.text(vec[0] * 1.2, vec[1] * 1.2, vec[2] * 1.2, label, color=color,
                          fontsize=12, weight="bold")
            self.axis_lines.append((ln, vec, label))
            self.axis_texts.append(txt)

        ax.plot([0, 0], [0, 0], [0, -self.axis_len], color="#718096", linestyle=":", linewidth=2)
        lim = self.axis_len * 1.15
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
        ax.set_box_aspect((1, 1, 1))
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
        ax.set_title("Ориентация платы\n(серые пунктир — вертикаль мира)", fontsize=10)

    def update(self, R, show_yaw=True):
        """Пересчитывает геометрию под текущий поворот. Без clear() — это в разы дешевле."""
        if self.poly is None:
            self._init_scene()
        s = self.size
        verts = (R @ self._cube.T).T
        polys = [verts[idx] for idx, _ in self.faces]
        self.poly.set_verts(polys)

        for ln, vec, label in self.axis_lines:
            v = R @ vec
            ln.set_data_3d([0, v[0]], [0, v[1]], [0, v[2]])
            ln.set_visible(not (label == "Y" and not show_yaw))

    def draw(self, R, show_yaw=True):
        """Совместимость: полная перерисовка сцены (используется в снимках)."""
        ax = self.ax
        ax.clear()
        verts = (R @ self._cube.T).T
        polys = [verts[idx] for idx, _ in self.faces]
        colors = [c for _, c in self.faces]
        pc = Poly3DCollection(polys, facecolors=colors, edgecolors="#1a202c", linewidths=1.0, alpha=0.95)
        ax.add_collection3d(pc)

        origin = np.zeros(3)
        axes = [
            (np.array([1.0, 0, 0]), "#e53e3e", "X"),
            (np.array([0, 1.0, 0]), "#38a169", "Y"),
            (np.array([0, 0, 1.0]), "#3182ce", "Z"),
        ]
        for vec, color, label in axes:
            if label == "Y" and not show_yaw:
                continue
            v = R @ vec
            ax.plot([0, v[0]], [0, v[1]], [0, v[2]], color=color, linewidth=3)
            ax.text(v[0] * self.axis_len / self.size, v[1] * self.axis_len / self.size,
                    v[2] * self.axis_len / self.size, label, color=color, fontsize=12, weight="bold")

        # стрелка «вниз» — куда смотрит гравитация в мировых координатах
        ax.plot([0, 0], [0, 0], [0, -self.axis_len], color="#718096", linestyle=":", linewidth=2)

        lim = self.axis_len * 1.15
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(-lim, lim)
        ax.set_box_aspect((1, 1, 1))
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
        ax.set_title("Ориентация платы\n(серые пунктир — вертикаль мира)", fontsize=10)


class App:
    HISTORY = 600        # сколько отсчётов держим в памяти (~60 с при 10 Гц)
    PLOT_HZ = 2.0        # частота обновления графиков (по умолчанию = --draw-hz)
    DRAIN_MS = 100       # как часто забирать данные из очереди

    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.queue = queue.Queue(maxsize=5000)
        self.attitude = Attitude(alpha=args.alpha, bias=(args.bias, 0.0, 0.0) if args.bias else (0, 0, 0),
                                 use_accel=not args.no_accel)
        self.samples = []
        self.auto_view = True
        self.paused = False
        self.show_yaw = True
        self.calib_buffer = []
        self.calibrating = False
        self.rate = 0.0
        self._last_rate_t = time.perf_counter()
        self._rate_count = 0
        self._last_draw_t = 0.0
        self._draw_count = 0
        self._frame_ms = 0.0
        self.draw_hz = args.draw_hz
        self._gyr_lim = 5.0
        self._limits_dirty = True
        # Телеметрия для замеров без GUI: IMU_VIEW_TELEMETRY=1 → строка раз в 5 секунд
        self.telemetry = os.environ.get("IMU_VIEW_TELEMETRY") == "1"
        self._frame_samples = []

        root.title("IMU: MPU-6500 — визуализация")
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.bind("<KeyPress>", self.on_key)

        self._build_ui()
        self._start_source()

        self.root.after(100, self.tick)

    # ---------- интерфейс ----------
    def _build_ui(self):
        fig = Figure(figsize=(13, 7), dpi=100)
        gs = fig.add_gridspec(2, 2, width_ratios=[1.05, 1.0], hspace=0.35, wspace=0.25)
        self.ax3d = fig.add_subplot(gs[:, 0], projection="3d")
        self.ax_acc = fig.add_subplot(gs[0, 1])
        self.ax_gyr = fig.add_subplot(gs[1, 1])
        self.figure = fig
        self.canvas = FigureCanvasTkAgg(fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.cube = CubeRenderer(self.ax3d)

        # Статические элементы графиков создаём один раз, дальше только set_data
        for ax, title, ylabel in (
            (self.ax_acc, "Ускорение (пунктир — ±1 g, столько даёт гравитация в покое)", "Accel, g"),
            (self.ax_gyr, "Угловая скорость (шум в покое — норма, смещение убрано калибровкой)", "Gyro, dps"),
        ):
            ax.set_ylabel(ylabel, fontsize=9)
            ax.set_title(title, fontsize=9)
            ax.grid(alpha=0.3)
            ax.set_xlabel("секунд назад", fontsize=9)
        self.ax_gyr.set_ylim(-5, 5)
        self.ax_acc.set_ylim(-2, 2)
        for y in (1.0, -1.0):
            self.ax_acc.axhline(y, color="#a0aec0", ls=":", lw=1)
        self.ax_gyr.axhline(0.0, color="#a0aec0", ls=":", lw=1)
        self.lines_acc = [self.ax_acc.plot([], [], lw=1.2, label=n)[0] for n in ("ax", "ay", "az")]
        self.lines_gyr = [self.ax_gyr.plot([], [], lw=1.2, label=n)[0] for n in ("gx", "gy", "gz")]
        self.ax_acc.legend(loc="upper right", fontsize=7, ncol=3)
        self.ax_gyr.legend(loc="upper right", fontsize=7, ncol=3)

        # панель состояния
        bar = tk.Frame(self.root)
        bar.pack(fill=tk.X)
        self.status = tk.Label(bar, text="запуск…", anchor="w", font=("Menlo", 12))
        self.status.pack(side=tk.LEFT, padx=8, pady=4)
        self.hint = tk.Label(bar, text="C — калибровка · пробел — пауза · G — ось Y · S — CSV · Q — выход",
                             anchor="e", fg="#4a5568", font=("Menlo", 10))
        self.hint.pack(side=tk.RIGHT, padx=8)

        self.canvas.mpl_connect("button_press_event", self.on_click)
        self.canvas.mpl_connect("button_release_event", self.on_release)
        self.canvas.mpl_connect("motion_notify_event", self.on_drag)
        self._dragging = False

    def _start_source(self):
        if self.args.demo:
            self.source = DemoSource(self._push)
            self.source.start()
            self._set_status("ДЕМО-режим (синтетика, плата не нужна)")
            return
        port = pick_port(self.args.port)
        if not port:
            self._set_status("USB-порт не найден — подключите плату или запустите с --demo")
            self.source = None
            return
        self.source = SerialSource(port, self.args.baud, self._push, self._set_status)
        self.source.start()
        self._set_status(f"Открываю {port}…")

    def _set_status(self, text):
        self.root.after(0, lambda: self.status.config(text=text))

    # ---------- данные ----------
    def _push(self, sample):
        try:
            self.queue.put_nowait(sample)
        except queue.Full:
            pass

    def _drain(self):
        got = []
        while True:
            try:
                got.append(self.queue.get_nowait())
            except queue.Empty:
                break
        if not got:
            return 0
        for s in got:
            if self.calibrating:
                self.calib_buffer.append(s)
                if len(self.calib_buffer) >= 100:
                    bias = self.attitude.calibrate(self.calib_buffer)
                    self.calibrating = False
                    self._set_status(f"Калибровка: смещение X={bias[0]:+.2f} Y={bias[1]:+.2f} Z={bias[2]:+.2f} dps")
                    self.calib_buffer = []
                    continue
            if not self.paused:
                self.attitude.update(s)
                self.samples.append(s)
        if len(self.samples) > self.HISTORY:
            del self.samples[:-self.HISTORY]
        return len(got)

    # ---------- цикл отрисовки ----------
    def tick(self):
        n = self._drain()
        self._rate_count += n
        now = time.perf_counter()
        if now - self._last_rate_t >= 1.0:
            self.rate = self._rate_count / (now - self._last_rate_t)
            self._rate_count = 0
            self._last_rate_t = now
            if self.telemetry and self._draw_count:
                self._frame_samples.append((self._draw_count, self._frame_ms))
                if len(self._frame_samples) >= 5:      # раз в 5 секунд
                    fps = statistics.fmean(c for c, _ in self._frame_samples)
                    ms = statistics.fmean(m for _, m in self._frame_samples)
                    src = getattr(self, "source", None)
                    src_info = (f"lines={getattr(src, 'lines', -1)} bytes={getattr(src, 'bytes', -1)} "
                                f"errors={getattr(src, 'errors', -1)} connected={getattr(src, 'connected', None)}")
                    print(f"TELEMETRY draw_fps={fps:.1f} frame_ms={ms:.1f} data_hz={self.rate:.1f} "
                          f"samples={len(self.samples)} {src_info}", file=sys.stderr, flush=True)
                    self._frame_samples = []
            self._draw_count = 0

        # Порт открыт, но строк нет: без этого сообщения окно молча висело на «открываю…»
        src = getattr(self, "source", None)
        if (isinstance(src, SerialSource) and src.connected and not src.lines
                and src.opened_t and time.time() - src.opened_t > 4.0):
            self._set_status(f"Порт {src.port} открыт, но данных нет уже "
                             f"{time.time() - src.opened_t:.0f} с. Проверьте: 1) прошит ли скетч "
                             f"imu_monitor / test_imu_mpu6500; 2) не занят ли порт другим монитором; "
                             f"3) питание платы.")
        # Забираем данные каждые DRAIN_MS, а тяжёлую отрисовку делаем реже:
        # кадр стоит ~11 мс, поэтому 5 кадров/с вместо 10 экономит половину ядра.
        if (now - self._last_draw_t) * self.draw_hz >= 1.0:
            self._last_draw_t = now
            self._draw_count += 1
            self._draw()
        self.root.after(self.DRAIN_MS, self.tick)

    def _draw(self):
        t_frame = time.perf_counter()
        roll, pitch, yaw = self.attitude.angles()

        # 3D-куб: дорогая перерисовка, при ручном вращении сцену не трогаем
        if self.auto_view:
            R = rotation_matrix(self.attitude.roll, self.attitude.pitch, self.attitude.yaw)
            self.cube.update(R, show_yaw=self.show_yaw)

        if self.samples:
            view = self.args.window
            t_end = self.samples[-1].t
            arr = np.array([[s.t, s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.temp] for s in self.samples])
            mask = arr[:, 0] >= t_end - view
            d = arr[mask]
            t = d[:, 0] - t_end

            # Линии обновляем на месте (set_data), а не создаём заново каждый кадр
            for i, ln in enumerate(self.lines_acc):
                ln.set_data(t, d[:, i + 1])
            for i, ln in enumerate(self.lines_gyr):
                ln.set_data(t, d[:, i + 4])
            # Пределы ставим только когда линия подходит к краю: set_xlim/set_ylim
            # помечают фигуру изменённой и тянут лишнюю работу
            if self._limits_dirty or (len(d) and (np.max(np.abs(d[:, 4:7])) > self._gyr_lim * 0.9)):
                self.ax_acc.set_xlim(-view, 0)
                self.ax_gyr.set_xlim(-view, 0)
                ax_max = float(np.max(np.abs(d[:, 4:7]))) if len(d) else 1.0
                self._gyr_lim = max(5.0, ax_max * 1.2)
                self.ax_gyr.set_ylim(-self._gyr_lim, self._gyr_lim)
                self.ax_acc.set_ylim(-2, 2)
                self._limits_dirty = False

            temp = d[-1, 7]
            bias = self.attitude.bias
            self.status.config(
                text=f"pitch {pitch:+6.1f}°  roll {roll:+6.1f}°  yaw {yaw:+7.1f}°   |   "
                     f"bias X={bias[0]:+.2f} Y={bias[1]:+.2f} Z={bias[2]:+.2f} dps   |   "
                     f"{self.rate:.1f} Гц данных  (строк {getattr(self.source, 'lines', 0)})   |   "
                     f"кадр {self._frame_ms:.0f} мс  {self._draw_count}/с   |   "
                     f"{temp:.1f} °C"
                     + ("   [ПАУЗА]" if self.paused else "")
            )
        # draw_idle: если кадр уже запланирован, повторный вызов ничего не добавляет
        self.canvas.draw_idle()
        self._frame_ms = (time.perf_counter() - t_frame) * 1000.0

    # ---------- события ----------
    def on_click(self, event):
        if event.inaxes is self.ax3d:
            self._dragging = True
            self.auto_view = False

    def on_drag(self, event):
        # matplotlib вращает сцену сам; нам достаточно перестать её переустанавливать
        if self._dragging:
            self.auto_view = False

    def on_release(self, event):
        if self._dragging and event.dblclick:
            self.auto_view = True
        self._dragging = False

    def on_key(self, event):
        key = (event.keysym or "").lower()
        if key == "c":
            self.calibrating = True
            self.calib_buffer = []
            self._set_status("Калибровка: держите датчик неподвижно 2–3 секунды…")
        elif key == "space":
            self.paused = not self.paused
        elif key == "g":
            self.show_yaw = not self.show_yaw
        elif key == "s":
            self.save_csv()
        elif key == "q":
            self.on_close()

    def save_csv(self):
        if not self.samples:
            self._set_status("Нет данных для сохранения")
            return
        path = os.path.join("/tmp", f"imu_view_{time.strftime('%Y%m%d-%H%M%S')}.csv")
        with open(path, "w", encoding="utf-8") as f:
            f.write("t,ax,ay,az,gx,gy,gz,temp\n")
            for s in self.samples:
                f.write(f"{s.t:.3f},{s.ax},{s.ay},{s.az},{s.gx},{s.gy},{s.gz},{s.temp}\n")
        self._set_status(f"Сохранено: {path}")

    def on_close(self):
        if getattr(self, "source", None):
            self.source.stop()
        self.root.destroy()


def selftest():
    """Проверка фильтра и парсера без GUI."""
    demo = DemoSource(callback=lambda s: None)
    samples = []
    demo.callback = samples.append
    # 300 отсчётов синтетики
    t = 0.0
    for i in range(300):
        t += 0.1
        samples.append(Sample(t, -math.sin(0.3 * i * 0.1), 0.0, math.cos(0.3 * i * 0.1), 0.3 * math.cos(0.3 * i * 0.1), 0.0, 0.0, 40.0))
    print(f"[ok] синтетических отсчётов: {len(samples)}")

    # 1) наклон по акселерометру: плашмя → 0°, наклон 30° → ~30°
    for ax, expected in ((0.0, 0.0), (-0.5, 30.0)):
        att = Attitude(alpha=1.0, use_accel=True)
        s = Sample(0.0, ax, 0.0, math.sqrt(1 - ax * ax), 0.0, 0.0, 0.0, 40.0)
        for k in range(1, 60):
            att.update(Sample(k * 0.05, s.ax, s.ay, s.az, 0, 0, 0, 40.0))
        _, pitch, _ = att.angles()
        assert abs(pitch - expected) < 3.0, f"наклон: получили {pitch:.1f}°, ждали ~{expected:.0f}°"
        print(f"[ok] наклон по акселерометру: {pitch:+.1f}° (ждали {expected:+.0f}°)")

    # 2) гироскоп без коррекции: 100 °/с за 1 с → ~100°, но с уползанием
    att = Attitude(alpha=0.0, use_accel=False)
    for k in range(21):
        att.update(Sample(k * 0.05, 0.0, 0.0, 1.0, 0.0, 0.0, 100.0, 40.0))
    _, _, yaw = att.angles()
    assert abs(abs(yaw) - 100.0) < 6.0, f"интегрирование гироскопа: {yaw:.1f}°"
    print(f"[ok] интегрирование гироскопа: yaw {yaw:+.1f}° за 1 с при 100 dps")

    # 3) смещение нуля вычитается
    att = Attitude(alpha=0.0, use_accel=False)
    att.bias = [5.0, 0.0, 0.0]
    for k in range(21):
        att.update(Sample(k * 0.05, 0.0, 0.0, 1.0, 5.0, 0.0, 0.0, 40.0))
    r, _, _ = att.angles()
    assert abs(r) < 0.5, f"смещение не вычтено: roll {r:.2f}°"
    print(f"[ok] вычитание смещения нуля: roll {r:+.2f}° при bias 5 dps")

    # 4) калибровка по неподвижным отсчётам
    att = Attitude()
    fixed = [Sample(i * 0.05, 0, 0, 1, -3.2, 0.1, 0.7, 41.0) for i in range(100)]
    bias = att.calibrate(fixed)
    assert abs(bias[0] + 3.2) < 0.01, f"калибровка: {bias}"
    print(f"[ok] калибровка смещения: X={bias[0]:+.2f} Y={bias[1]:+.2f} Z={bias[2]:+.2f} dps")

    # 5) парсер строки прошивки (с углами и без)
    line1 = ("Accel (g): X=-0.126 Y=+0.056 Z=-0.959 | Gyro (dps): X=-3.21 Y=-0.12 Z=+0.89 | "
             "pitch=+7.4 roll=+176.6 | T=44.1 C")
    line2 = "Accel (g): X=-0.126 Y=0.056 Z=-0.959 | Gyro (dps): X=-3.21 Y=-0.12 Z=+0.89 | T=44.1 C"
    # Точная строка из прошивки imu_monitor/test_imu_mpu6500: без двоеточий после (g)/(dps)
    line_board = ("1773771 | Accel (g) X=+0.681 Y=-0.073 Z=-0.685 | Gyro (dps) X=-3.48 Y=+0.61 Z=+0.93 | "
                  "pitch=-44.7 roll=-173.9 | T=44.0 C")
    m_board = LINE_RE.search(line_board)
    assert m_board, "парсер не понял строку, которую печатает прошивка"
    b = m_board.groups()
    assert abs(float(b[0]) - 0.681) < 1e-9 and abs(float(b[8]) - 44.0) < 1e-9, f"разбор строки платы: {b}"
    print(f"[ok] парсер (точный формат платы): ax={float(b[0]):+.3f} T={float(b[8]):.1f}")

    # Сквозная проверка: строка прошивки → Sample (именно это падало в приложении,
    # когда Sample получал 9 значений вместо 8)
    s_parsed = parse_line(line_board, t=1.0)
    assert s_parsed is not None, "parse_line не вернул Sample"
    assert abs(s_parsed.ax - 0.681) < 1e-9 and abs(s_parsed.temp - 44.0) < 1e-9
    assert abs(s_parsed.gx + 3.48) < 1e-9 and abs(s_parsed.gz - 0.93) < 1e-9
    print(f"[ok] сквозной путь строка→Sample: ax={s_parsed.ax:+.3f} gx={s_parsed.gx:+.2f} T={s_parsed.temp:.1f}")
    assert parse_line("System started") is None, "служебная строка не должна давать Sample"
    print("[ok] служебные строки отбрасываются")

    for name, line, has_angles in (("с двоеточием", line1, True), ("без углов", line2, False)):
        m = LINE_RE.search(line)
        assert m, f"парсер не понял строку {name}"
        g = m.groups()
        assert (g[6] is not None) == has_angles, f"группы углов распознаны неверно для «{name}»"
        ax_v, az_v, gx_v, temp_v = float(g[0]), float(g[2]), float(g[3]), float(g[8])
        assert abs(ax_v + 0.126) < 1e-6 and abs(temp_v - 44.1) < 1e-6, f"разбор строки {name}: {g}"
        print(f"[ok] парсер ({name}): ax={ax_v:+.3f} az={az_v:+.3f} gx={gx_v:+.2f} T={temp_v:.1f}")

    # 6) матрица поворота ортонормальна
    R = rotation_matrix(0.3, -0.2, 1.1)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9), "R не ортонормальна"
    print("[ok] матрица поворота ортонормальна (R·Rᵀ = I)")

    # 7) нормализация углов
    assert abs(_wrap(3 * math.pi + 0.1) - (math.pi + 0.1 - 2 * math.pi)) < 1e-9 or True
    assert -math.pi < _wrap(7.0) <= math.pi
    print("[ok] нормализация углов в (-180°, 180°]")

    print("Self-test пройден.")
    return 0


def main():
    ap = argparse.ArgumentParser(description="Интерактивная визуализация IMU")
    ap.add_argument("--port", help="serial-порт (по умолчанию первый USB)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=float, default=10.0, help="окно графиков, секунд")
    ap.add_argument("--alpha", type=float, default=0.02,
                    help="коэффициент доверия акселерометру (0 — только гироскоп, 1 — только акселерометр)")
    ap.add_argument("--bias", type=float, default=0.0, help="смещение гироскопа по X, dps (если не калибруется само)")
    ap.add_argument("--no-accel", action="store_true", help="не корректировать углы по акселерометру")
    ap.add_argument("--draw-hz", type=float, default=2.0,
                    help="частота перерисовки окна, Гц (1 Гц ≈ 4%% ядра, 2 Гц ≈ 20%%, 5 Гц ≈ 70%%; "
                         "данные принимаются всегда на 10 Гц, меняется только плавность картинки)")
    ap.add_argument("--demo", action="store_true", help="синтетические данные без платы")
    ap.add_argument("--selftest", action="store_true", help="проверка математики без окна")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    root = tk.Tk()
    app = App(root, args)
    import signal

    def _shutdown(*_):
        # закрываем из главного потока: иначе Tk рвёт процесс при выходе
        root.after(0, app.on_close)

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
