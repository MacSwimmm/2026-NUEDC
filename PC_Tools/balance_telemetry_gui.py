#!/usr/bin/env python3
"""F407-Balance 遥测桌面监控界面（只读，不下发电机或 PID 命令）。"""

from __future__ import annotations

import csv
import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Any

import balance_telemetry as core


APP_DIR = Path(__file__).resolve().parent
LOG_DIR = APP_DIR / "logs"

COLORS = {
    "bg": "#f2f5f9",
    "surface": "#ffffff",
    "border": "#dbe3ec",
    "text": "#142033",
    "muted": "#66758a",
    "blue": "#2563eb",
    "green": "#159447",
    "amber": "#d97706",
    "red": "#dc2626",
    "purple": "#7c3aed",
    "cyan": "#0891b2",
    "grid": "#e8edf3",
}


class SerialWorker(threading.Thread):
    def __init__(
        self,
        device: str,
        baud: int,
        events: queue.Queue[tuple[Any, ...]],
        stop_event: threading.Event,
    ) -> None:
        super().__init__(daemon=True)
        self.device = device
        self.baud = baud
        self.events = events
        self.stop_event = stop_event

    def run(self) -> None:
        serial, _list_ports = core.import_serial()
        ser = None
        try:
            ser = serial.Serial(self.device, self.baud, timeout=0.20)
            self.events.put(("connected", self.device, self.baud))
            while not self.stop_event.is_set():
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                try:
                    kind, data = core.parse_frame(line)
                except core.FrameError as exc:
                    self.events.put(("bad_frame", str(exc), line[:160]))
                    continue
                self.events.put(("frame", kind, data, line))
        except Exception as exc:  # 串口拔出、占用和驱动错误都回到 UI 显示。
            self.events.put(("serial_error", str(exc)))
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
            self.events.put(("stopped",))


class MetricCard(tk.Frame):
    def __init__(self, parent: tk.Misc, title: str) -> None:
        super().__init__(
            parent,
            bg=COLORS["surface"],
            highlightbackground=COLORS["border"],
            highlightthickness=1,
            padx=14,
            pady=10,
        )
        self.title_label = tk.Label(
            self,
            text=title,
            bg=COLORS["surface"],
            fg=COLORS["muted"],
            font=("Microsoft YaHei UI", 9),
            anchor="w",
        )
        self.title_label.pack(fill="x")
        self.value_label = tk.Label(
            self,
            text="--",
            bg=COLORS["surface"],
            fg=COLORS["text"],
            font=("Microsoft YaHei UI", 16, "bold"),
            anchor="w",
        )
        self.value_label.pack(fill="x", pady=(3, 0))
        self.detail_label = tk.Label(
            self,
            text="等待数据",
            bg=COLORS["surface"],
            fg=COLORS["muted"],
            font=("Microsoft YaHei UI", 8),
            anchor="w",
        )
        self.detail_label.pack(fill="x", pady=(2, 0))

    def set(self, value: str, detail: str = "", tone: str = "text") -> None:
        self.value_label.configure(text=value, fg=COLORS.get(tone, COLORS["text"]))
        self.detail_label.configure(text=detail or " ")


class TrendChart(tk.Canvas):
    def __init__(
        self,
        parent: tk.Misc,
        title: str,
        series: list[tuple[str, str, str]],
        y_min: float,
        y_max: float,
        unit: str,
        thresholds: list[tuple[float, str, str]] | None = None,
    ) -> None:
        super().__init__(
            parent,
            bg=COLORS["surface"],
            highlightbackground=COLORS["border"],
            highlightthickness=1,
            height=270,
        )
        self.title = title
        self.series = series
        self.y_min = y_min
        self.y_max = y_max
        self.unit = unit
        self.thresholds = thresholds or []
        self.samples: list[dict[str, Any]] = []
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_samples(self, samples: list[dict[str, Any]]) -> None:
        self.samples = samples
        self.redraw()

    def _y(self, value: float, top: float, height: float) -> float:
        clipped = min(self.y_max, max(self.y_min, value))
        return top + (self.y_max - clipped) / (self.y_max - self.y_min) * height

    def redraw(self) -> None:
        self.delete("all")
        width = max(300, self.winfo_width())
        height = max(200, self.winfo_height())
        left, right, top, bottom = 58.0, 18.0, 48.0, 30.0
        plot_w = width - left - right
        plot_h = height - top - bottom

        self.create_text(
            16,
            15,
            text=self.title,
            anchor="nw",
            fill=COLORS["text"],
            font=("Microsoft YaHei UI", 11, "bold"),
        )

        legend_x = width - 16
        for label, _field, color in reversed(self.series):
            text_id = self.create_text(
                legend_x,
                17,
                text=label,
                anchor="ne",
                fill=COLORS["muted"],
                font=("Microsoft YaHei UI", 8),
            )
            bbox = self.bbox(text_id)
            if bbox:
                legend_x = bbox[0] - 20
                self.create_line(legend_x, 21, legend_x + 13, 21, fill=color, width=3)
                legend_x -= 7

        for index in range(5):
            ratio = index / 4.0
            y = top + ratio * plot_h
            value = self.y_max - ratio * (self.y_max - self.y_min)
            self.create_line(left, y, left + plot_w, y, fill=COLORS["grid"])
            self.create_text(
                left - 8,
                y,
                text=f"{value:g}",
                anchor="e",
                fill=COLORS["muted"],
                font=("Consolas", 8),
            )
        self.create_text(
            10,
            top - 12,
            text=self.unit,
            anchor="w",
            fill=COLORS["muted"],
            font=("Microsoft YaHei UI", 8),
        )

        for value, color, dash_name in self.thresholds:
            dash = (5, 4) if dash_name == "dash" else (2, 3)
            y = self._y(value, top, plot_h)
            self.create_line(left, y, left + plot_w, y, fill=color, dash=dash)

        if len(self.samples) < 2:
            self.create_text(
                left + plot_w / 2,
                top + plot_h / 2,
                text="连接串口后显示最近 30 秒趋势",
                fill=COLORS["muted"],
                font=("Microsoft YaHei UI", 10),
            )
            return

        ticks = [int(item.get("tick_ms", 0)) for item in self.samples]
        first_tick, last_tick = ticks[0], ticks[-1]
        span = max(1, last_tick - first_tick)
        for label, field, color in self.series:
            points: list[float] = []
            for sample, tick in zip(self.samples, ticks):
                value = sample.get(field, "")
                if value == "":
                    continue
                x = left + (tick - first_tick) / span * plot_w
                y = self._y(float(value), top, plot_h)
                points.extend((x, y))
            if len(points) >= 4:
                self.create_line(*points, fill=color, width=2, smooth=False)

        seconds = span / 1000.0
        self.create_text(
            left,
            top + plot_h + 17,
            text=f"-{seconds:.1f} s",
            anchor="w",
            fill=COLORS["muted"],
            font=("Consolas", 8),
        )
        self.create_text(
            left + plot_w,
            top + plot_h + 17,
            text="现在",
            anchor="e",
            fill=COLORS["muted"],
            font=("Microsoft YaHei UI", 8),
        )


class TrackSensorCanvas(tk.Canvas):
    def __init__(self, parent: tk.Misc) -> None:
        super().__init__(
            parent,
            bg=COLORS["surface"],
            highlightbackground=COLORS["border"],
            highlightthickness=1,
            height=126,
        )
        self.bits = 0
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_bits(self, bits: int) -> None:
        self.bits = bits & 0xFF
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        width = max(520, self.winfo_width())
        self.create_text(
            16,
            14,
            text="八路循迹传感器（bit0=C1，1=检测到黑线）",
            anchor="nw",
            fill=COLORS["text"],
            font=("Microsoft YaHei UI", 11, "bold"),
        )
        self.create_text(
            width - 16,
            17,
            text=f"mask 0x{self.bits:02X}",
            anchor="ne",
            fill=COLORS["muted"],
            font=("Consolas", 10, "bold"),
        )
        gap = 10.0
        left = 18.0
        right = 18.0
        cell_width = (width - left - right - gap * 7.0) / 8.0
        top, bottom = 48.0, 101.0
        for index in range(8):
            x0 = left + index * (cell_width + gap)
            x1 = x0 + cell_width
            active = bool(self.bits & (1 << index))
            fill = "#111827" if active else "#f8fafc"
            outline = COLORS["blue"] if active else COLORS["border"]
            text_color = "white" if active else COLORS["muted"]
            self.create_rectangle(
                x0,
                top,
                x1,
                bottom,
                fill=fill,
                outline=outline,
                width=2,
            )
            self.create_text(
                (x0 + x1) / 2.0,
                (top + bottom) / 2.0,
                text=f"C{index + 1}",
                fill=text_color,
                font=("Microsoft YaHei UI", 10, "bold"),
            )


class BalanceTelemetryApp:
    def __init__(self, root: tk.Tk, smoke_test: bool = False) -> None:
        self.root = root
        self.smoke_test = smoke_test
        self.events: queue.Queue[tuple[Any, ...]] = queue.Queue()
        self.worker: SerialWorker | None = None
        self.stop_event: threading.Event | None = None
        self.session_active = False
        self.disconnect_requested = False

        self.csv_handle = None
        self.csv_writer: csv.DictWriter | None = None
        self.output_path: Path | None = None
        self.summary_path: Path | None = None
        self.track_csv_handle = None
        self.track_csv_writer: csv.DictWriter | None = None
        self.track_output_path: Path | None = None
        self.output_custom = False

        self.samples: deque[dict[str, Any]] = deque(maxlen=2400)
        self.plot_samples: deque[dict[str, Any]] = deque(maxlen=600)
        self.track_samples: deque[dict[str, Any]] = deque(maxlen=2400)
        self.track_plot_samples: deque[dict[str, Any]] = deque(maxlen=600)
        self.cfg: dict[str, Any] = {}
        self.rows = 0
        self.track_rows = 0
        self.bad_frames = 0
        self.sequence_gaps = 0
        self.track_sequence_gaps = 0
        self.last_sequence: int | None = None
        self.last_track_sequence: int | None = None
        self.last_tel_wall = 0.0
        self.last_track_wall = 0.0
        self.last_advice_wall = 0.0
        self.last_log_sample_wall = 0.0
        self.last_track_log_wall = 0.0
        self.last_plot_wall = 0.0
        self.last_track_plot_wall = 0.0
        self.last_report = ""
        self.port_map: dict[str, str] = {}

        self._configure_root()
        self._build_ui()
        self.refresh_ports()
        self._set_disconnected_ui()
        self.root.after(40, self._poll_events)
        self.root.after(1000, self._heartbeat)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _configure_root(self) -> None:
        self.root.title("F407-Balance 平衡与循迹遥测助手")
        self.root.geometry("1460x900")
        self.root.minsize(1120, 720)
        self.root.configure(bg=COLORS["bg"])
        try:
            self.root.tk.call("tk", "scaling", 1.15)
        except tk.TclError:
            pass
        self.root.option_add("*Font", ("Microsoft YaHei UI", 9))

        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TFrame", background=COLORS["bg"])
        style.configure("TLabel", background=COLORS["bg"], foreground=COLORS["text"])
        style.configure("TNotebook", background=COLORS["bg"], borderwidth=0)
        style.configure(
            "TNotebook.Tab",
            padding=(18, 9),
            font=("Microsoft YaHei UI", 9, "bold"),
        )
        style.map("TNotebook.Tab", background=[("selected", COLORS["surface"])])
        style.configure("Treeview", rowheight=28, fieldbackground=COLORS["surface"])
        style.configure("Treeview.Heading", font=("Microsoft YaHei UI", 9, "bold"))

    def _build_ui(self) -> None:
        self.root.grid_rowconfigure(2, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        top = tk.Frame(self.root, bg="#17233a", padx=20, pady=13)
        top.grid(row=0, column=0, sticky="ew")
        top.grid_columnconfigure(1, weight=1)
        tk.Label(
            top,
            text="F407-Balance",
            bg="#17233a",
            fg="white",
            font=("Microsoft YaHei UI", 18, "bold"),
        ).grid(row=0, column=0, sticky="w")
        tk.Label(
            top,
            text="平衡与循迹遥测 · 双 CSV 记录 · 调参诊断（只读）",
            bg="#17233a",
            fg="#aebbd0",
            font=("Microsoft YaHei UI", 10),
        ).grid(row=1, column=0, sticky="w")
        self.connection_badge = tk.Label(
            top,
            text="● 未连接",
            bg="#17233a",
            fg="#94a3b8",
            font=("Microsoft YaHei UI", 11, "bold"),
        )
        self.connection_badge.grid(row=0, column=2, rowspan=2, sticky="e")

        controls = tk.Frame(
            self.root,
            bg=COLORS["surface"],
            highlightbackground=COLORS["border"],
            highlightthickness=1,
            padx=14,
            pady=10,
        )
        controls.grid(row=1, column=0, sticky="ew", padx=14, pady=(12, 8))
        controls.grid_columnconfigure(5, weight=1)

        tk.Label(controls, text="串口", bg=COLORS["surface"], fg=COLORS["muted"]).grid(
            row=0, column=0, sticky="w"
        )
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            controls, textvariable=self.port_var, state="readonly", width=38
        )
        self.port_combo.grid(row=0, column=1, padx=(6, 6), sticky="w")
        ttk.Button(controls, text="刷新", command=self.refresh_ports).grid(
            row=0, column=2, padx=(0, 14)
        )

        tk.Label(controls, text="波特率", bg=COLORS["surface"], fg=COLORS["muted"]).grid(
            row=0, column=3, sticky="w"
        )
        self.baud_var = tk.StringVar(value=str(core.DEFAULT_BAUD))
        self.baud_combo = ttk.Combobox(
            controls,
            textvariable=self.baud_var,
            values=("115200",),
            state="readonly",
            width=10,
        )
        self.baud_combo.grid(row=0, column=4, padx=(6, 14), sticky="w")

        self.connect_button = tk.Button(
            controls,
            text="连接并开始记录",
            command=self.toggle_connection,
            bg=COLORS["blue"],
            fg="white",
            activebackground="#1d4ed8",
            activeforeground="white",
            relief="flat",
            padx=18,
            pady=7,
            font=("Microsoft YaHei UI", 9, "bold"),
            cursor="hand2",
        )
        self.connect_button.grid(row=0, column=6, padx=(8, 0), sticky="e")

        tk.Label(controls, text="保存到", bg=COLORS["surface"], fg=COLORS["muted"]).grid(
            row=1, column=0, pady=(9, 0), sticky="w"
        )
        self.output_var = tk.StringVar(value=str(self._new_default_output()))
        self.output_entry = ttk.Entry(controls, textvariable=self.output_var)
        self.output_entry.grid(
            row=1, column=1, columnspan=5, padx=(6, 6), pady=(9, 0), sticky="ew"
        )
        ttk.Button(controls, text="选择文件", command=self.choose_output).grid(
            row=1, column=6, padx=(8, 0), pady=(9, 0), sticky="e"
        )
        ttk.Button(controls, text="打开目录", command=self.open_log_folder).grid(
            row=1, column=7, padx=(8, 0), pady=(9, 0), sticky="e"
        )

        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=2, column=0, sticky="nsew", padx=14, pady=(0, 8))
        self.dashboard_tab = tk.Frame(self.notebook, bg=COLORS["bg"])
        self.track_tab = tk.Frame(self.notebook, bg=COLORS["bg"])
        self.diagnostic_tab = tk.Frame(self.notebook, bg=COLORS["bg"])
        self.log_tab = tk.Frame(self.notebook, bg=COLORS["bg"])
        self.notebook.add(self.dashboard_tab, text="平衡监控")
        self.notebook.add(self.track_tab, text="循迹监控")
        self.notebook.add(self.diagnostic_tab, text="参数与诊断")
        self.notebook.add(self.log_tab, text="运行日志")

        self._build_dashboard()
        self._build_track_dashboard()
        self._build_diagnostics()
        self._build_log()

        status = tk.Frame(self.root, bg="#e7ecf3", padx=14, pady=6)
        status.grid(row=3, column=0, sticky="ew")
        status.grid_columnconfigure(0, weight=1)
        self.status_var = tk.StringVar(value="请选择串口并连接")
        tk.Label(
            status,
            textvariable=self.status_var,
            bg="#e7ecf3",
            fg=COLORS["muted"],
            anchor="w",
        ).grid(row=0, column=0, sticky="ew")
        self.counter_var = tk.StringVar(value="样本 0 · 坏帧 0 · 序号缺口 0")
        tk.Label(
            status,
            textvariable=self.counter_var,
            bg="#e7ecf3",
            fg=COLORS["muted"],
            anchor="e",
            font=("Consolas", 9),
        ).grid(row=0, column=1, sticky="e")

    def _build_dashboard(self) -> None:
        self.dashboard_tab.grid_columnconfigure((0, 1, 2, 3), weight=1, uniform="card")
        self.dashboard_tab.grid_rowconfigure(2, weight=1)

        card_specs = [
            ("state", "控制状态"),
            ("position", "小球位置"),
            ("velocity", "小球速度"),
            ("error", "位置误差"),
            ("camera", "视觉链路"),
            ("motor", "请求 / ACK"),
            ("fault", "安全状态"),
            ("rate", "遥测频率"),
        ]
        self.cards: dict[str, MetricCard] = {}
        for index, (key, title) in enumerate(card_specs):
            card = MetricCard(self.dashboard_tab, title)
            card.grid(
                row=index // 4,
                column=index % 4,
                sticky="nsew",
                padx=(0 if index % 4 == 0 else 6, 0 if index % 4 == 3 else 6),
                pady=(4, 5),
            )
            self.cards[key] = card

        chart_frame = tk.Frame(self.dashboard_tab, bg=COLORS["bg"])
        chart_frame.grid(row=2, column=0, columnspan=4, sticky="nsew", pady=(5, 0))
        chart_frame.grid_columnconfigure(0, weight=1)
        chart_frame.grid_rowconfigure((0, 1), weight=1)

        self.position_chart = TrendChart(
            chart_frame,
            "小球位置与误差（最近 30 秒）",
            [
                ("位置", "position_mm", COLORS["blue"]),
                ("误差", "error_mm", COLORS["purple"]),
            ],
            -120.0,
            120.0,
            "mm",
            [
                (0.0, COLORS["muted"], "dot"),
                (50.0, COLORS["amber"], "dash"),
                (-50.0, COLORS["amber"], "dash"),
                (10.0, COLORS["green"], "dash"),
                (-10.0, COLORS["green"], "dash"),
            ],
        )
        self.position_chart.grid(row=0, column=0, sticky="nsew", pady=(0, 5))

        self.pulse_chart = TrendChart(
            chart_frame,
            "电机目标与 ACK（机械边界 -340 / +330 pulse）",
            [
                ("控制目标", "command_position_pulse", COLORS["blue"]),
                ("最终请求", "final_requested_pulse", COLORS["amber"]),
                ("已ACK", "last_acked_pulse", COLORS["green"]),
            ],
            -340.0,
            330.0,
            "pulse",
            [
                (0.0, COLORS["muted"], "dot"),
                (-340.0, COLORS["red"], "dash"),
                (330.0, COLORS["red"], "dash"),
            ],
        )
        self.pulse_chart.grid(row=1, column=0, sticky="nsew", pady=(5, 0))

    def _build_track_dashboard(self) -> None:
        self.track_tab.grid_columnconfigure((0, 1, 2, 3), weight=1, uniform="track_card")
        self.track_tab.grid_rowconfigure(3, weight=1)

        card_specs = [
            ("state", "循迹状态"),
            ("mode", "车辆模式"),
            ("error", "循迹误差"),
            ("time", "任务计时"),
            ("left_speed", "左轮目标 / 反馈"),
            ("right_speed", "右轮目标 / 反馈"),
            ("pwm", "左右 PWM"),
            ("safety", "编码器与终点"),
        ]
        self.track_cards: dict[str, MetricCard] = {}
        for index, (key, title) in enumerate(card_specs):
            card = MetricCard(self.track_tab, title)
            card.grid(
                row=index // 4,
                column=index % 4,
                sticky="nsew",
                padx=(0 if index % 4 == 0 else 6, 0 if index % 4 == 3 else 6),
                pady=(4, 5),
            )
            self.track_cards[key] = card

        self.track_sensor_canvas = TrackSensorCanvas(self.track_tab)
        self.track_sensor_canvas.grid(
            row=2, column=0, columnspan=4, sticky="ew", pady=(5, 5)
        )

        chart_frame = tk.Frame(self.track_tab, bg=COLORS["bg"])
        chart_frame.grid(row=3, column=0, columnspan=4, sticky="nsew", pady=(5, 0))
        chart_frame.grid_columnconfigure(0, weight=1)
        chart_frame.grid_columnconfigure(1, weight=2)
        chart_frame.grid_columnconfigure(2, weight=1)
        chart_frame.grid_rowconfigure(0, weight=1)

        self.track_error_chart = TrendChart(
            chart_frame,
            "循迹加权误差",
            [("误差", "line_error", COLORS["purple"])],
            -400.0,
            400.0,
            "weight",
            [(0.0, COLORS["muted"], "dot")],
        )
        self.track_error_chart.grid(row=0, column=0, sticky="nsew", padx=(0, 5))

        self.track_speed_chart = TrendChart(
            chart_frame,
            "左右轮目标与反馈",
            [
                ("左目标", "left_speed_set_count_10ms", COLORS["blue"]),
                ("左反馈", "left_speed_count_10ms", COLORS["cyan"]),
                ("右目标", "right_speed_set_count_10ms", COLORS["red"]),
                ("右反馈", "right_speed_count_10ms", COLORS["amber"]),
            ],
            -40.0,
            40.0,
            "count / 10 ms",
            [(0.0, COLORS["muted"], "dot")],
        )
        self.track_speed_chart.grid(row=0, column=1, sticky="nsew", padx=5)

        self.track_pwm_chart = TrendChart(
            chart_frame,
            "左右轮 PWM",
            [
                ("左PWM", "left_pwm", COLORS["blue"]),
                ("右PWM", "right_pwm", COLORS["red"]),
            ],
            -160.0,
            160.0,
            "PWM",
            [(0.0, COLORS["muted"], "dot")],
        )
        self.track_pwm_chart.grid(
            row=0, column=2, sticky="nsew", padx=(5, 0)
        )

    def _build_diagnostics(self) -> None:
        self.diagnostic_tab.grid_columnconfigure(0, weight=0)
        self.diagnostic_tab.grid_columnconfigure(1, weight=1)
        self.diagnostic_tab.grid_rowconfigure(1, weight=1)

        params = tk.LabelFrame(
            self.diagnostic_tab,
            text=" 当前控制参数（只读） ",
            bg=COLORS["surface"],
            fg=COLORS["text"],
            padx=12,
            pady=10,
        )
        params.grid(row=0, column=0, rowspan=2, sticky="nsw", padx=(0, 8), pady=4)
        param_specs = [
            ("Kp", "cfg_kp"),
            ("Ki", "cfg_ki"),
            ("Kd", "cfg_kd"),
            ("目标跟随增益", "cfg_centering_gain_s"),
            ("速度旧值权重", "cfg_velocity_old_weight"),
            ("积分窗口", "cfg_integral_window_mm"),
            ("最大推进速度", "cfg_max_slew_pulse_s"),
            ("单帧最大步长", "cfg_max_step_pulse"),
            ("软件范围", "soft_range"),
            ("硬件范围", "hard_range"),
            ("视觉新鲜度", "cfg_camera_fresh_ms"),
            ("视觉继续时间", "cfg_vision_continue_ms"),
            ("轨迹目标", "trajectory_targets"),
            ("轨迹启动门", "trajectory_gate"),
            ("轨迹 PID 倍率", "trajectory_pid_scales"),
            ("轨迹速率倍率", "trajectory_rate_scales"),
            ("评分阈值", "trajectory_score"),
        ]
        self.param_vars: dict[str, tk.StringVar] = {}
        for row, (label, key) in enumerate(param_specs):
            tk.Label(
                params,
                text=label,
                bg=COLORS["surface"],
                fg=COLORS["muted"],
                anchor="w",
                width=15,
            ).grid(row=row, column=0, sticky="w", pady=4)
            var = tk.StringVar(value="--")
            self.param_vars[key] = var
            tk.Label(
                params,
                textvariable=var,
                bg=COLORS["surface"],
                fg=COLORS["text"],
                anchor="e",
                width=18,
                font=("Consolas", 10, "bold"),
            ).grid(row=row, column=1, sticky="e", pady=4)

        advice_frame = tk.LabelFrame(
            self.diagnostic_tab,
            text=" 实时分析与建议（不会自动改参数） ",
            bg=COLORS["surface"],
            fg=COLORS["text"],
            padx=8,
            pady=8,
        )
        advice_frame.grid(row=0, column=1, sticky="nsew", padx=(8, 0), pady=4)
        advice_frame.grid_rowconfigure(0, weight=1)
        advice_frame.grid_columnconfigure(0, weight=1)
        self.advice_text = tk.Text(
            advice_frame,
            wrap="word",
            bg="#fbfcfe",
            fg=COLORS["text"],
            relief="flat",
            padx=10,
            pady=8,
            height=15,
        )
        self.advice_text.grid(row=0, column=0, sticky="nsew")
        self.advice_text.insert("1.0", "连接并采集至少 1~2 秒后生成实时建议。")
        self.advice_text.configure(state="disabled")

        diag_frame = tk.LabelFrame(
            self.diagnostic_tab,
            text=" 通信、视觉与安全计数 ",
            bg=COLORS["surface"],
            fg=COLORS["text"],
            padx=8,
            pady=8,
        )
        diag_frame.grid(row=1, column=1, sticky="nsew", padx=(8, 0), pady=(8, 4))
        diag_frame.grid_rowconfigure(0, weight=1)
        diag_frame.grid_columnconfigure(0, weight=1)
        self.diag_tree = ttk.Treeview(
            diag_frame, columns=("value", "meaning"), show="headings", height=10
        )
        self.diag_tree.heading("value", text="当前值")
        self.diag_tree.heading("meaning", text="说明")
        self.diag_tree.column("value", width=110, anchor="e")
        self.diag_tree.column("meaning", width=470, anchor="w")
        self.diag_tree.grid(row=0, column=0, sticky="nsew")
        self.diag_fields = [
            ("camera_parse_error_count", "视觉文本解析错误"),
            ("camera_physical_reject_count", "视觉坐标超过 ±120 mm"),
            ("camera_uart_error_count", "USART2 HAL 错误"),
            ("motor_tx_fail_count", "USART1 电机发送失败"),
            ("motor_ack_timeout_count", "电机 ACK 超时"),
            ("motor_ack_reject_count", "电机明确拒绝"),
            ("hard_limit_reject_count", "-340/+330 硬限位拒绝"),
            ("vision_hold_count", "进入 VISION_HOLD 次数"),
            ("tuning_error_mask", "调参校验掩码"),
        ]
        for key, meaning in self.diag_fields:
            self.diag_tree.insert("", "end", iid=key, values=("--", meaning))

    def _build_log(self) -> None:
        self.log_tab.grid_rowconfigure(0, weight=1)
        self.log_tab.grid_columnconfigure(0, weight=1)
        frame = tk.Frame(
            self.log_tab,
            bg=COLORS["surface"],
            highlightbackground=COLORS["border"],
            highlightthickness=1,
        )
        frame.grid(row=0, column=0, sticky="nsew", pady=4)
        frame.grid_rowconfigure(0, weight=1)
        frame.grid_columnconfigure(0, weight=1)
        self.log_text = tk.Text(
            frame,
            wrap="word",
            bg="#111827",
            fg="#d1d5db",
            insertbackground="white",
            relief="flat",
            padx=12,
            pady=10,
            font=("Consolas", 9),
        )
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self._log("界面已启动。请选择 WCH-Link 对应串口后连接。")

    def _new_default_output(self) -> Path:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return LOG_DIR / f"balance_telemetry_{stamp}.csv"

    def refresh_ports(self) -> None:
        try:
            _serial, list_ports = core.import_serial()
            ports = list(list_ports.comports())
        except SystemExit as exc:
            messagebox.showerror("串口组件", str(exc))
            return

        current_device = self.port_map.get(self.port_var.get())
        ports.sort(
            key=lambda item: (
                "bluetooth" in item.description.lower() or "蓝牙" in item.description,
                item.device,
            )
        )
        self.port_map.clear()
        labels = []
        selected_label = ""
        for port in ports:
            label = f"{port.device}  ·  {port.description}"
            self.port_map[label] = port.device
            labels.append(label)
            if port.device == current_device:
                selected_label = label

        self.port_combo.configure(values=labels)
        if selected_label:
            self.port_var.set(selected_label)
        elif labels:
            preferred = next(
                (label for label in labels if "WCH-Link" in label or "USB" in label),
                labels[0],
            )
            self.port_var.set(preferred)
        else:
            self.port_var.set("")
        self._log(f"刷新串口：发现 {len(labels)} 个设备。")

    def choose_output(self) -> None:
        initial = Path(self.output_var.get()) if self.output_var.get() else self._new_default_output()
        selected = filedialog.asksaveasfilename(
            title="选择遥测 CSV 保存位置",
            initialdir=str(initial.parent),
            initialfile=initial.name,
            defaultextension=".csv",
            filetypes=(("CSV 文件", "*.csv"), ("所有文件", "*.*")),
        )
        if selected:
            self.output_var.set(selected)
            self.output_custom = True

    def open_log_folder(self) -> None:
        path = self.output_path.parent if self.output_path else Path(self.output_var.get()).parent
        path.mkdir(parents=True, exist_ok=True)
        try:
            if os.name == "nt":
                os.startfile(path)  # type: ignore[attr-defined]
            elif sys.platform == "darwin":
                subprocess.Popen(["open", str(path)])
            else:
                subprocess.Popen(["xdg-open", str(path)])
        except Exception as exc:
            messagebox.showerror("打开目录失败", str(exc))

    def toggle_connection(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            self.request_disconnect()
        else:
            self.start_connection()

    def start_connection(self) -> None:
        label = self.port_var.get()
        device = self.port_map.get(label)
        if not device:
            messagebox.showwarning("请选择串口", "请先在下拉框选择一个串口。")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showwarning("波特率", "波特率必须是整数。")
            return

        self._reset_session_data()
        self.disconnect_requested = False
        self.stop_event = threading.Event()
        self.worker = SerialWorker(device, baud, self.events, self.stop_event)
        self.worker.start()
        self.connect_button.configure(state="disabled", text="正在连接…")
        self.connection_badge.configure(text="● 正在连接", fg=COLORS["amber"])
        self.status_var.set(f"正在打开 {device}…")
        self._log(f"请求连接 {device} @ {baud}。")

    def request_disconnect(self) -> None:
        self.disconnect_requested = True
        if self.stop_event is not None:
            self.stop_event.set()
        self.connect_button.configure(state="disabled", text="正在停止…")
        self.status_var.set("正在停止采集并生成分析报告…")
        self._log("用户请求停止采集。")

    def _prepare_output(self) -> None:
        path = Path(self.output_var.get()).expanduser()
        if not path.is_absolute():
            path = (APP_DIR / path).resolve()
        if path.suffix.lower() != ".csv":
            path = path.with_suffix(".csv")
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists():
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = path.with_name(f"{path.stem}_{stamp}{path.suffix}")
        self.output_path = path
        self.summary_path = path.with_name(path.stem + "_summary.txt")
        self.track_output_path = path.with_name(path.stem + "_track.csv")
        self.output_var.set(str(path))
        try:
            self.csv_handle = path.open("w", newline="", encoding="utf-8-sig")
            self.csv_writer = csv.DictWriter(self.csv_handle, fieldnames=core.CSV_FIELDS)
            self.csv_writer.writeheader()
            self.track_csv_handle = self.track_output_path.open(
                "w", newline="", encoding="utf-8-sig"
            )
            self.track_csv_writer = csv.DictWriter(
                self.track_csv_handle, fieldnames=core.TRACK_CSV_FIELDS
            )
            self.track_csv_writer.writeheader()
        except Exception:
            self._close_output()
            raise

    def _reset_session_data(self) -> None:
        self._close_output()
        self.samples.clear()
        self.plot_samples.clear()
        self.track_samples.clear()
        self.track_plot_samples.clear()
        self.cfg.clear()
        self.rows = 0
        self.track_rows = 0
        self.bad_frames = 0
        self.sequence_gaps = 0
        self.track_sequence_gaps = 0
        self.last_sequence = None
        self.last_track_sequence = None
        self.last_tel_wall = 0.0
        self.last_track_wall = 0.0
        self.last_advice_wall = 0.0
        self.last_log_sample_wall = 0.0
        self.last_track_log_wall = 0.0
        self.last_plot_wall = 0.0
        self.last_track_plot_wall = 0.0
        self.last_report = ""
        if not self.output_custom:
            self.output_var.set(str(self._new_default_output()))
        self._clear_dashboard()

    def _poll_events(self) -> None:
        processed = 0
        while processed < 250:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            processed += 1
            kind = event[0]
            if kind == "connected":
                self._on_connected(str(event[1]), int(event[2]))
            elif kind == "frame":
                self._on_frame(str(event[1]), dict(event[2]), str(event[3]))
            elif kind == "bad_frame":
                self.bad_frames += 1
                if self.bad_frames <= 5 or self.bad_frames % 20 == 0:
                    self._log(f"坏帧 #{self.bad_frames}: {event[1]} | {event[2]}")
            elif kind == "serial_error":
                self._log(f"串口错误：{event[1]}")
                self.status_var.set(f"串口错误：{event[1]}")
                self.connection_badge.configure(text="● 串口错误", fg=COLORS["red"])
            elif kind == "stopped":
                self._on_stopped()
        self._update_counters()
        self.root.after(40, self._poll_events)

    def _on_connected(self, device: str, baud: int) -> None:
        try:
            self._prepare_output()
        except Exception as exc:
            if self.stop_event is not None:
                self.stop_event.set()
            messagebox.showerror("无法创建 CSV", str(exc))
            return
        self.session_active = True
        self.connect_button.configure(
            state="normal",
            text="停止并生成报告",
            bg=COLORS["red"],
            activebackground="#b91c1c",
        )
        self.port_combo.configure(state="disabled")
        self.baud_combo.configure(state="disabled")
        self.output_entry.configure(state="disabled")
        self.connection_badge.configure(text="● 已连接，等待遥测", fg=COLORS["amber"])
        self.status_var.set(
            f"{device} 已打开；等待 HELLO/TEL/TRK。平衡 CSV: {self.output_path}"
        )
        self._log(
            f"已连接 {device} @ {baud}，平衡记录 {self.output_path}，"
            f"循迹记录 {self.track_output_path}。"
        )

    def _on_frame(self, kind: str, data: dict[str, Any], raw_line: str) -> None:
        if kind == "HELLO":
            self.connection_badge.configure(text="● 设备在线", fg=COLORS["green"])
            self._log(
                f"HELLO: {data.get('product')}，协议 v{data.get('protocol_version')}，"
                f"周期 {data.get('sample_period_ms')} ms。"
            )
            return
        if kind == "CFG":
            self.cfg.update(data)
            self._update_parameter_panel()
            self._log(
                "CFG: "
                f"Kp={self.cfg.get('cfg_kp')} Ki={self.cfg.get('cfg_ki')} "
                f"Kd={self.cfg.get('cfg_kd')} max_slew={self.cfg.get('cfg_max_slew_pulse_s')} "
                f"max_step={self.cfg.get('cfg_max_step_pulse')}。"
            )
            return
        if kind == "TRK":
            self._on_track_frame(data)
            return
        if kind != "TEL":
            return

        self.last_tel_wall = time.monotonic()
        self.connection_badge.configure(text="● 正在接收遥测", fg=COLORS["green"])
        row = dict(data)
        row.update({name: self.cfg.get(name, "") for name in core.CFG_OUTPUT_FIELDS})
        if self.csv_writer is not None:
            self.csv_writer.writerow(row)
        self.rows += 1
        if self.csv_handle is not None and self.rows % 20 == 0:
            self.csv_handle.flush()

        sequence = int(row.get("sequence", 0))
        if self.last_sequence is not None and sequence > self.last_sequence + 1:
            self.sequence_gaps += sequence - self.last_sequence - 1
        self.last_sequence = sequence
        self.samples.append(row)
        self.plot_samples.append(row)
        self._update_dashboard(row)

        now = time.monotonic()
        if now - self.last_advice_wall >= 3.0:
            self._update_advice()
            self.last_advice_wall = now
        if now - self.last_log_sample_wall >= 2.0:
            self._log(
                f"TEL seq={sequence} state={core.STATE_NAMES.get(int(row['state']), row['state'])} "
                f"pos={float(row['position_mm']):+.1f} mm "
                f"vel={float(row['velocity_mm_s']):+.1f} mm/s "
                f"req/ack={int(row['final_requested_pulse']):+d}/"
                f"{int(row['last_acked_pulse']):+d}。"
            )
            self.last_log_sample_wall = now

    def _on_track_frame(self, data: dict[str, Any]) -> None:
        self.last_track_wall = time.monotonic()
        self.connection_badge.configure(text="● 正在接收遥测", fg=COLORS["green"])
        row = dict(data)
        if self.track_csv_writer is not None:
            self.track_csv_writer.writerow(row)
        self.track_rows += 1
        if self.track_csv_handle is not None and self.track_rows % 20 == 0:
            self.track_csv_handle.flush()

        sequence = int(row.get("sequence", 0))
        if (
            self.last_track_sequence is not None
            and sequence > self.last_track_sequence + 1
        ):
            self.track_sequence_gaps += sequence - self.last_track_sequence - 1
        self.last_track_sequence = sequence
        self.track_samples.append(row)
        self.track_plot_samples.append(row)
        self._update_track_dashboard(row)

        now = time.monotonic()
        if now - self.last_track_log_wall >= 2.0:
            state = core.TRACK_STATE_NAMES.get(int(row.get("state", -1)), row.get("state"))
            self._log(
                f"TRK seq={sequence} state={state} mask=0x{int(row.get('track_bits', 0)):02X} "
                f"err={int(row.get('line_error', 0)):+d} "
                f"speed L/R={float(row.get('left_speed_count_10ms', 0.0)):+.1f}/"
                f"{float(row.get('right_speed_count_10ms', 0.0)):+.1f}。"
            )
            self.last_track_log_wall = now

    def _on_stopped(self) -> None:
        report_path = self._finish_session()
        self.worker = None
        self.stop_event = None
        self.session_active = False
        self._set_disconnected_ui()
        if report_path:
            self.status_var.set(f"采集已停止；报告已保存：{report_path}")
            self._log(f"分析报告已保存：{report_path}")
        elif self.disconnect_requested:
            self.status_var.set("采集已停止，未收到有效 TEL 样本。")
        else:
            self.status_var.set("串口连接已结束。")

    def _finish_session(self) -> Path | None:
        report_path = None
        if self.rows > 0 and self.summary_path is not None:
            self.last_report = core.render_report(list(self.samples), self.cfg)
            try:
                self.summary_path.write_text(self.last_report + "\n", encoding="utf-8-sig")
                report_path = self.summary_path
            except Exception as exc:
                self._log(f"报告保存失败：{exc}")
        self._close_output()
        return report_path

    def _close_output(self) -> None:
        if self.csv_handle is not None:
            try:
                self.csv_handle.flush()
                self.csv_handle.close()
            except Exception:
                pass
        self.csv_handle = None
        self.csv_writer = None
        if self.track_csv_handle is not None:
            try:
                self.track_csv_handle.flush()
                self.track_csv_handle.close()
            except Exception:
                pass
        self.track_csv_handle = None
        self.track_csv_writer = None

    def _set_disconnected_ui(self) -> None:
        self.connect_button.configure(
            state="normal",
            text="连接并开始记录",
            bg=COLORS["blue"],
            activebackground="#1d4ed8",
        )
        self.port_combo.configure(state="readonly")
        self.baud_combo.configure(state="readonly")
        self.output_entry.configure(state="normal")
        self.connection_badge.configure(text="● 未连接", fg="#94a3b8")

    def _clear_dashboard(self) -> None:
        for card in getattr(self, "cards", {}).values():
            card.set("--", "等待数据")
        for card in getattr(self, "track_cards", {}).values():
            card.set("--", "等待 TRK 数据")
        if hasattr(self, "position_chart"):
            self.position_chart.set_samples([])
            self.pulse_chart.set_samples([])
        if hasattr(self, "track_sensor_canvas"):
            self.track_sensor_canvas.set_bits(0)
            self.track_error_chart.set_samples([])
            self.track_speed_chart.set_samples([])
            self.track_pwm_chart.set_samples([])
        if hasattr(self, "advice_text"):
            self._set_text(self.advice_text, "连接并采集至少 1~2 秒后生成实时建议。")
        if hasattr(self, "diag_tree"):
            for key, meaning in self.diag_fields:
                self.diag_tree.item(key, values=("--", meaning))

    def _update_dashboard(self, row: dict[str, Any]) -> None:
        state = int(row.get("state", -1))
        state_name = core.STATE_NAMES.get(state, str(state))
        trajectory_started = bool(int(row.get("trajectory_started", 0)))
        trajectory_completed = bool(int(row.get("trajectory_completed", 0)))
        trajectory_ready = bool(int(row.get("trajectory_ready", 0)))
        if trajectory_completed:
            state_tone = "green" if int(row.get("score_pass", 0)) else "red"
            state_detail = (
                f"轨迹 {'PASS' if int(row.get('score_pass', 0)) else 'FAIL'} · "
                f"{int(row.get('completion_time_ms', 0)) / 1000.0:.3f} s"
            )
        elif trajectory_started:
            state_tone = "blue" if state != core.VISION_HOLD_STATE else "amber"
            state_detail = f"轨迹 {int(row.get('run_elapsed_ms', 0)) / 1000.0:.3f} s"
        elif state == core.HOLD_ORIGIN_STATE:
            state_tone = "green"
            ready_count = int(row.get("trajectory_ready_count", 0))
            state_detail = "PA0 轨迹 READY" if trajectory_ready else f"O点确认 {ready_count} 帧"
        elif state == core.TARGET_LEVELING_STATE:
            state_tone = "amber"
            state_detail = "PE2：电机轴回水平"
        elif state == core.TARGET_WAIT_CAPTURE_STATE:
            state_tone = "amber"
            state_detail = "PE2：放球后再按一次记录坐标"
        elif state == core.HOLD_CAPTURED_TARGET_STATE:
            state_tone = "green"
            state_detail = "PE2 目标锁定"
        else:
            state_tone = "amber" if state == core.VISION_HOLD_STATE else "blue"
            state_detail = f"run {int(row.get('tick_ms', 0)) / 1000.0:.1f} s"
        self.cards["state"].set(
            state_name,
            state_detail,
            state_tone,
        )
        target_mm = int(row.get("target_mm", 0))
        self.cards["position"].set(
            f"{float(row.get('position_mm', 0.0)):+.1f} mm",
            f"目标 {target_mm:+d} mm",
            "blue",
        )
        self.cards["velocity"].set(
            f"{float(row.get('velocity_mm_s', 0.0)):+.1f} mm/s",
            f"采样 {int(row.get('sample_dt_ms', 0))} ms",
            "cyan",
        )
        error = float(row.get("error_mm", 0.0))
        self.cards["error"].set(
            f"{error:+.1f} mm",
            "评分最大允许误差 ±10 mm",
            "green" if abs(error) <= 10.0 else "amber",
        )

        online = bool(row.get("camera_online"))
        found = bool(row.get("target_found"))
        age = int(row.get("last_valid_age_ms", 0))
        if online and found:
            camera_value, camera_tone = "ONLINE", "green"
        elif online:
            camera_value, camera_tone = "未识别到球", "amber"
        else:
            camera_value, camera_tone = "OFFLINE", "red"
        self.cards["camera"].set(camera_value, f"有效坐标年龄 {age} ms", camera_tone)

        requested = int(row.get("final_requested_pulse", 0))
        acked = int(row.get("last_acked_pulse", 0))
        self.cards["motor"].set(
            f"{requested:+d} / {acked:+d}",
            f"TX {int(row.get('last_transmitted_pulse', 0)):+d} pulse",
            "green" if requested == acked else "amber",
        )
        fault = int(row.get("fault", 0))
        tuning_mask = int(row.get("tuning_error_mask", 0))
        if fault:
            self.cards["fault"].set(
                core.FAULT_NAMES.get(fault, str(fault)), "控制故障已出现", "red"
            )
        elif tuning_mask:
            self.cards["fault"].set(
                f"TUNE 0x{tuning_mask:08X}", "参数校验未通过", "amber"
            )
        else:
            self.cards["fault"].set("NONE", "未见故障或调参拒绝", "green")

        rate = self._sample_rate()
        self.cards["rate"].set(
            f"{rate:.1f} Hz" if rate else "--",
            f"seq {int(row.get('sequence', 0))}",
            "green" if 16.0 <= rate <= 24.0 else "amber",
        )

        now = time.monotonic()
        if now - self.last_plot_wall >= 0.20:
            self.position_chart.set_samples(list(self.plot_samples))
            self.pulse_chart.set_samples(list(self.plot_samples))
            for key, meaning in self.diag_fields:
                value = int(row.get(key, 0))
                display = f"0x{value:08X}" if key == "tuning_error_mask" else str(value)
                self.diag_tree.item(key, values=(display, meaning))
            self.last_plot_wall = now
        self.status_var.set(
            f"正在记录 {self.output_path} · {state_name} · pos {float(row['position_mm']):+.1f} mm"
        )

    def _update_track_dashboard(self, row: dict[str, Any]) -> None:
        state = int(row.get("state", -1))
        mode = int(row.get("car_mode", -1))
        state_name = core.TRACK_STATE_NAMES.get(state, str(state))
        mode_name = core.TRACK_CAR_MODE_NAMES.get(mode, str(mode))
        initialized = bool(int(row.get("initialized", 0)))
        track_bits = int(row.get("track_bits", 0)) & 0xFF
        line_error = int(row.get("line_error", 0))

        self.track_cards["state"].set(
            state_name,
            f"seq {int(row.get('sequence', 0))} · {self._track_sample_rate():.1f} Hz",
            "green" if initialized else "amber",
        )
        self.track_cards["mode"].set(
            mode_name,
            "控制已初始化" if initialized else "等待循迹初始化",
            "green" if mode == 1 else "amber",
        )
        self.track_cards["error"].set(
            f"{line_error:+d}",
            f"mask 0x{track_bits:02X}",
            "green" if abs(line_error) <= 50 else "amber",
        )
        elapsed_ms = int(row.get("elapsed_time_ms", 0))
        final_ms = int(row.get("final_time_ms", 0))
        self.track_cards["time"].set(
            f"{elapsed_ms / 1000.0:.3f} s",
            f"最终 {final_ms / 1000.0:.3f} s" if final_ms else "任务进行中/未开始",
            "blue",
        )

        left_target = float(row.get("left_speed_set_count_10ms", 0.0))
        left_speed = float(row.get("left_speed_count_10ms", 0.0))
        right_target = float(row.get("right_speed_set_count_10ms", 0.0))
        right_speed = float(row.get("right_speed_count_10ms", 0.0))
        self.track_cards["left_speed"].set(
            f"{left_target:+.1f} / {left_speed:+.1f}",
            "目标 / 反馈，count/10ms",
            "cyan",
        )
        self.track_cards["right_speed"].set(
            f"{right_target:+.1f} / {right_speed:+.1f}",
            "目标 / 反馈，count/10ms",
            "cyan",
        )
        left_pwm = int(row.get("left_pwm", 0))
        right_pwm = int(row.get("right_pwm", 0))
        self.track_cards["pwm"].set(
            f"{left_pwm:+d} / {right_pwm:+d}",
            "左 / 右",
            "blue",
        )

        left_fault = bool(int(row.get("left_encoder_fault", 0)))
        right_fault = bool(int(row.get("right_encoder_fault", 0)))
        finish_pending = bool(int(row.get("finish_pending", 0)))
        finish_armed = bool(int(row.get("finish_armed", 0)))
        if left_fault or right_fault:
            safety_value = f"ENC L{int(left_fault)} R{int(right_fault)}"
            safety_detail = "编码器反馈故障"
            safety_tone = "red"
        else:
            safety_value = "OK"
            safety_detail = (
                f"终点 armed={int(finish_armed)} pending={int(finish_pending)}"
            )
            safety_tone = "green" if not finish_pending else "amber"
        self.track_cards["safety"].set(safety_value, safety_detail, safety_tone)

        self.track_sensor_canvas.set_bits(track_bits)
        now = time.monotonic()
        if now - self.last_track_plot_wall >= 0.20:
            samples = list(self.track_plot_samples)
            self.track_error_chart.set_samples(samples)
            self.track_speed_chart.set_samples(samples)
            self.track_pwm_chart.set_samples(samples)
            self.last_track_plot_wall = now

    def _sample_rate(self) -> float:
        if len(self.samples) < 2:
            return 0.0
        first = self.samples[max(0, len(self.samples) - 21)]
        last = self.samples[-1]
        tick_delta = int(last["tick_ms"]) - int(first["tick_ms"])
        seq_delta = int(last["sequence"]) - int(first["sequence"])
        return seq_delta * 1000.0 / tick_delta if tick_delta > 0 else 0.0

    def _track_sample_rate(self) -> float:
        if len(self.track_samples) < 2:
            return 0.0
        first = self.track_samples[max(0, len(self.track_samples) - 21)]
        last = self.track_samples[-1]
        tick_delta = int(last["tick_ms"]) - int(first["tick_ms"])
        seq_delta = int(last["sequence"]) - int(first["sequence"])
        return seq_delta * 1000.0 / tick_delta if tick_delta > 0 else 0.0

    def _update_parameter_panel(self) -> None:
        units = {
            "cfg_integral_window_mm": " mm",
            "cfg_max_slew_pulse_s": " pulse/s",
            "cfg_max_step_pulse": " pulse/frame",
            "cfg_camera_fresh_ms": " ms",
            "cfg_vision_continue_ms": " ms",
        }
        for key, var in self.param_vars.items():
            if key == "soft_range":
                low = self.cfg.get("cfg_soft_min_pulse", "--")
                high = self.cfg.get("cfg_soft_max_pulse", "--")
                var.set(f"{low} .. {high} pulse")
            elif key == "hard_range":
                low = self.cfg.get("cfg_hard_min_pulse", "--")
                high = self.cfg.get("cfg_hard_max_pulse", "--")
                var.set(f"{low} .. {high} pulse")
            elif key == "trajectory_targets":
                positive = self.cfg.get("cfg_positive_target_mm", "--")
                negative = self.cfg.get("cfg_negative_target_mm", "--")
                var.set(f"{positive} -> {negative} mm")
            elif key == "trajectory_gate":
                position = self.cfg.get(
                    "cfg_trajectory_start_position_tolerance_mm", "--"
                )
                speed = self.cfg.get(
                    "cfg_trajectory_start_speed_tolerance_mm_s", "--"
                )
                samples = self.cfg.get("cfg_trajectory_start_confirm_samples", "--")
                var.set(f"±{position} mm / {speed} mm/s / {samples}帧")
            elif key == "trajectory_pid_scales":
                kp = self.cfg.get("cfg_trajectory_kp_scale", "--")
                ki = self.cfg.get("cfg_trajectory_ki_scale", "--")
                kd = self.cfg.get("cfg_trajectory_kd_scale", "--")
                var.set(f"{kp} / {ki} / {kd}")
            elif key == "trajectory_rate_scales":
                follow = self.cfg.get("cfg_trajectory_follow_gain_scale", "--")
                slew = self.cfg.get("cfg_trajectory_slew_scale", "--")
                step = self.cfg.get("cfg_trajectory_step_scale", "--")
                var.set(f"{follow} / {slew} / {step}")
            elif key == "trajectory_score":
                duration = self.cfg.get("cfg_score_limit_ms", "--")
                error = self.cfg.get("cfg_official_error_mm", "--")
                var.set(f"{duration} ms / ±{error} mm")
            else:
                value = self.cfg.get(key, "--")
                var.set(f"{value}{units.get(key, '')}")

    def _update_advice(self) -> None:
        report = core.render_report(list(self.samples), self.cfg)
        self.last_report = report
        self._set_text(self.advice_text, report)

    def _update_counters(self) -> None:
        self.counter_var.set(
            f"平衡 {self.rows} · 循迹 {self.track_rows} · 坏帧 {self.bad_frames} · "
            f"缺口 B/T {self.sequence_gaps}/{self.track_sequence_gaps}"
        )

    def _heartbeat(self) -> None:
        if self.session_active:
            latest_wall = max(self.last_tel_wall, self.last_track_wall)
            age = time.monotonic() - latest_wall if latest_wall else 999.0
            if age > 2.0:
                self.connection_badge.configure(text="● 已连接但无遥测", fg=COLORS["red"])
                device = self.port_map.get(self.port_var.get(), self.port_var.get())
                self.status_var.set(
                    f"{device} 已打开，但 2 秒内没有 TEL/TRK；请检查 PB10→RX、共地和固件。"
                )
        self.root.after(1000, self._heartbeat)

    def _set_text(self, widget: tk.Text, content: str) -> None:
        widget.configure(state="normal")
        widget.delete("1.0", "end")
        widget.insert("1.0", content)
        widget.configure(state="disabled")

    def _log(self, message: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{stamp}] {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def on_close(self) -> None:
        if self.stop_event is not None:
            self.stop_event.set()
        if self.worker is not None and self.worker.is_alive():
            self.worker.join(timeout=0.5)
        self._finish_session()
        self.root.destroy()


def launch_gui(smoke_test: bool = False) -> int:
    root = tk.Tk()
    app = BalanceTelemetryApp(root, smoke_test=smoke_test)
    if smoke_test:
        root.withdraw()
        root.update_idletasks()
        kind, sample = core.parse_frame(
            core.make_test_tel(
                sequence=1,
                tick_ms=50,
                position_mm=18,
                velocity_x100=-900,
                error_x100=-1800,
            )
        )
        assert kind == "TEL"
        app.samples.append(sample)
        app.plot_samples.append(sample)
        app._update_dashboard(sample)
        track_kind, track_sample = core.parse_frame(
            core.make_test_track(
                sequence=2,
                tick_ms=100,
                track_bits=0x18,
                line_error=-50,
                left_speed_x100=2850,
                right_speed_x100=3025,
                left_speed_set_x100=3000,
                right_speed_set_x100=3000,
                left_pwm=81,
                right_pwm=76,
            )
        )
        assert track_kind == "TRK"
        app._on_track_frame(track_sample)
        assert app.track_rows == 1
        assert app.track_sensor_canvas.bits == 0x18
        root.update_idletasks()
        root.destroy()
        print("gui self-test passed")
        return 0
    root.mainloop()
    return 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        raise SystemExit(launch_gui(smoke_test=True))
    raise SystemExit(launch_gui())
