#!/usr/bin/env python3
"""MSPM0G3507 wheel-speed PI serial tuner for the TianMengXing CH340 UART0."""

from __future__ import annotations

import argparse
import csv
import math
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports


BAUD_RATE = 115200
KEEPALIVE_SECONDS = 0.20
PLOT_WINDOW_SECONDS = 10.0
AUTO_TRIAL_SECONDS = 2.2
AUTO_SETTLE_SECONDS = 0.25
AUTO_MAX_ITERATIONS = 8


@dataclass
class Sample:
    host_time: float
    board_ms: int
    left_target: int
    left_feedback: int
    left_output: int
    right_target: int
    right_feedback: int
    right_output: int
    left_count: int
    right_count: int


@dataclass(frozen=True)
class AutoTuneConfig:
    """从 Tk 主线程一次性取得的整定参数快照。"""

    side: str
    target: int
    left_kp: int
    right_kp: int
    left_ki: int
    right_ki: int
    left_limit: int
    right_limit: int


class SerialWorker:
    def __init__(self, lines: queue.Queue[str]) -> None:
        self.lines = lines
        self.port: serial.Serial | None = None
        self.reader: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        port = self.port
        return port is not None and port.is_open

    def connect(self, port_name: str) -> None:
        self.close(send_stop=False)
        stop_event = threading.Event()
        port = serial.Serial(port_name, BAUD_RATE, timeout=0.10, write_timeout=0.25)
        port.reset_input_buffer()
        self.stop_event = stop_event
        self.port = port
        self.reader = threading.Thread(
            target=self._read_loop, args=(port, stop_event), daemon=True
        )
        self.reader.start()

    def _read_loop(self, port: serial.Serial, stop_event: threading.Event) -> None:
        try:
            while not stop_event.is_set():
                try:
                    raw = port.readline()
                    if raw:
                        self.lines.put(raw.decode("ascii", errors="replace").strip())
                except (serial.SerialException, OSError) as exc:
                    if not stop_event.is_set():
                        self.lines.put(f"LOCAL,SERIAL_ERROR,{exc}")
                    break
        finally:
            try:
                port.close()
            except Exception:
                pass
            if self.port is port:
                self.port = None

    def send(self, command: str) -> None:
        data = (command.strip() + "\n").encode("ascii")
        with self.write_lock:
            port = self.port
            if port is None or not port.is_open:
                raise RuntimeError("串口尚未连接")
            port.write(data)
            port.flush()

    def close(self, send_stop: bool = True) -> None:
        if self.connected and send_stop:
            try:
                self.send("STOP")
            except Exception:
                pass
        stop_event = self.stop_event
        port = self.port
        reader = self.reader
        stop_event.set()
        self.port = None
        self.reader = None
        if port is not None:
            try:
                port.close()
            except Exception:
                pass
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=0.30)


class TraceCanvas(tk.Canvas):
    def __init__(self, master: tk.Misc, title: str, **kwargs) -> None:
        super().__init__(master, background="#11151b", highlightthickness=0, **kwargs)
        self.title = title

    def draw(self, samples: list[Sample], mode: str, limit: int) -> None:
        self.delete("all")
        width = max(self.winfo_width(), 400)
        height = max(self.winfo_height(), 160)
        left, top, right, bottom = 52, 22, width - 12, height - 28
        self.create_text(10, 8, anchor="nw", text=self.title, fill="#e6edf3")
        self.create_rectangle(left, top, right, bottom, outline="#44515f")
        if not samples:
            self.create_text(width / 2, height / 2, text="等待遥测数据", fill="#8696a7")
            return

        latest = samples[-1].host_time
        earliest = latest - PLOT_WINDOW_SECONDS
        visible = [sample for sample in samples if sample.host_time >= earliest]
        if len(visible) < 2:
            return

        if mode == "speed":
            series = [
                ("左目标", "#ffb454", [s.left_target for s in visible]),
                ("左反馈", "#ff5d73", [s.left_feedback for s in visible]),
                ("右目标", "#52d6ff", [s.right_target for s in visible]),
                ("右反馈", "#658cff", [s.right_feedback for s in visible]),
            ]
            maximum = max(100, max(abs(value) for _, _, values in series for value in values))
            maximum = int(math.ceil(maximum / 25.0) * 25)
            unit = "mm/s"
        else:
            series = [
                ("左PWM", "#ff5d73", [s.left_output for s in visible]),
                ("右PWM", "#658cff", [s.right_output for s in visible]),
            ]
            maximum = max(200, limit, max(abs(value) for _, _, values in series for value in values))
            maximum = int(math.ceil(maximum / 100.0) * 100)
            unit = "counts"

        def x_of(sample: Sample) -> float:
            return left + (sample.host_time - earliest) / PLOT_WINDOW_SECONDS * (right - left)

        def y_of(value: int) -> float:
            return top + (maximum - value) / (2.0 * maximum) * (bottom - top)

        for fraction in (-1.0, -0.5, 0.0, 0.5, 1.0):
            value = int(maximum * fraction)
            y = y_of(value)
            self.create_line(left, y, right, y, fill="#25303a")
            self.create_text(left - 6, y, anchor="e", text=str(value), fill="#8b9aab")
        self.create_text(left, bottom + 8, anchor="nw", text=f"最近 {PLOT_WINDOW_SECONDS:g}s / {unit}", fill="#8b9aab")

        legend_x = left
        for label, color, values in series:
            points: list[float] = []
            for sample, value in zip(visible, values):
                points.extend((x_of(sample), y_of(value)))
            if len(points) >= 4:
                self.create_line(*points, fill=color, width=2, smooth=False)
            self.create_line(legend_x, 13, legend_x + 18, 13, fill=color, width=3)
            self.create_text(legend_x + 22, 13, anchor="w", text=label, fill="#d3dce6")
            legend_x += 88


class PIDTunerApp:
    def __init__(self, root: tk.Tk, preferred_port: str | None) -> None:
        self.root = root
        self.root.title("天猛星 G3507 轮速 PI 调参器")
        self.root.geometry("1120x820")
        self.lines: queue.Queue[str] = queue.Queue()
        self.ui_events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.serial = SerialWorker(self.lines)
        self.samples: deque[Sample] = deque(maxlen=6000)
        self.samples_lock = threading.Lock()
        self.armed = False
        self.autotune_active = False
        self.autotune_cancel = threading.Event()
        self.last_keepalive = 0.0
        self.log_file = None
        self.log_writer = None

        self.port_var = tk.StringVar(value=preferred_port or "")
        self.connection_var = tk.StringVar(value="未连接")
        self.status_var = tk.StringVar(value="车轮必须架空；连接后先确认 ARM。")
        self.left_kp = tk.StringVar(value="400")
        self.right_kp = tk.StringVar(value="400")
        self.left_ki = tk.StringVar(value="400")
        self.right_ki = tk.StringVar(value="400")
        self.left_limit = tk.StringVar(value="1200")
        self.right_limit = tk.StringVar(value="1200")
        self.left_target = tk.StringVar(value="80")
        self.right_target = tk.StringVar(value="80")
        self.auto_side = tk.StringVar(value="L")

        self._build_ui()
        self.rescan_ports()
        if preferred_port:
            self.port_var.set(preferred_port)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(20, self.poll)

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="串口").pack(side="left")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=34)
        self.port_box.pack(side="left", padx=5)
        ttk.Button(top, text="刷新", command=self.rescan_ports).pack(side="left")
        ttk.Button(top, text="连接", command=self.connect).pack(side="left", padx=5)
        ttk.Button(top, text="断开", command=self.disconnect).pack(side="left")
        ttk.Label(top, textvariable=self.connection_var).pack(side="left", padx=12)

        controls = ttk.LabelFrame(self.root, text="参数与安全控制", padding=8)
        controls.pack(fill="x", padx=8, pady=4)
        headers = ["", "左轮", "右轮"]
        for column, text in enumerate(headers):
            ttk.Label(controls, text=text).grid(row=0, column=column, padx=6, pady=2)
        fields = [
            ("Kp", self.left_kp, self.right_kp),
            ("Ki", self.left_ki, self.right_ki),
            ("PWM限幅", self.left_limit, self.right_limit),
            ("目标 mm/s", self.left_target, self.right_target),
        ]
        for row, (label, left_var, right_var) in enumerate(fields, start=1):
            ttk.Label(controls, text=label).grid(row=row, column=0, sticky="e", padx=6)
            ttk.Entry(controls, textvariable=left_var, width=10).grid(row=row, column=1, padx=6, pady=2)
            ttk.Entry(controls, textvariable=right_var, width=10).grid(row=row, column=2, padx=6, pady=2)

        button_frame = ttk.Frame(controls)
        button_frame.grid(row=1, column=3, rowspan=4, padx=18, sticky="ns")
        ttk.Button(button_frame, text="应用 PI 参数", command=self.apply_params).pack(fill="x", pady=2)
        ttk.Button(button_frame, text="确认架空并 ARM", command=self.arm).pack(fill="x", pady=2)
        ttk.Button(button_frame, text="发送目标", command=self.send_target).pack(fill="x", pady=2)
        ttk.Button(button_frame, text="紧急停止", command=self.emergency_stop).pack(fill="x", pady=2)

        auto_frame = ttk.Frame(controls)
        auto_frame.grid(row=1, column=4, rowspan=4, padx=12, sticky="ns")
        ttk.Label(auto_frame, text="自动整定轮").pack()
        ttk.Radiobutton(auto_frame, text="左轮", value="L", variable=self.auto_side).pack(anchor="w")
        ttk.Radiobutton(auto_frame, text="右轮", value="R", variable=self.auto_side).pack(anchor="w")
        ttk.Button(auto_frame, text="开始受限自动整定", command=self.start_autotune).pack(fill="x", pady=4)
        ttk.Button(auto_frame, text="取消自动整定", command=self.cancel_autotune).pack(fill="x", pady=2)

        self.speed_plot = TraceCanvas(self.root, "目标速度与编码器反馈")
        self.speed_plot.pack(fill="both", expand=True, padx=8, pady=4)
        self.pwm_plot = TraceCanvas(self.root, "PI 输出 PWM（车辆相对方向）")
        self.pwm_plot.pack(fill="both", expand=True, padx=8, pady=4)

        bottom = ttk.Frame(self.root, padding=8)
        bottom.pack(fill="x")
        ttk.Label(bottom, textvariable=self.status_var).pack(side="left")
        self.console = tk.Text(bottom, height=6, width=70, state="disabled")
        self.console.pack(side="right", fill="x", expand=True, padx=(12, 0))

    def rescan_ports(self) -> None:
        ports = list(list_ports.comports())
        values = [f"{port.device} | {port.description}" for port in ports]
        self.port_box["values"] = values
        if not self.port_var.get() and values:
            preferred = next((value for value in values if "CH340" in value.upper() or "USB-SERIAL" in value.upper()), values[0])
            self.port_var.set(preferred)

    def _port_name(self) -> str:
        return self.port_var.get().split("|", 1)[0].strip()

    def connect(self) -> None:
        try:
            port_name = self._port_name()
            if not port_name:
                raise RuntimeError("没有选择串口")
            self.serial.connect(port_name)
            self.connection_var.set(f"已连接 {port_name} @ {BAUD_RATE}")
            self._open_log()
            self.serial.send("PING")
            self.serial.send("STREAM 1")
            self.serial.send("STATUS")
            self.status_var.set("已连接；电机保持停止，ARM 前必须架空车轮。")
        except Exception as exc:
            messagebox.showerror("连接失败", str(exc))

    def disconnect(self) -> None:
        self.autotune_cancel.set()
        self.serial.close(send_stop=True)
        self.armed = False
        self.connection_var.set("未连接")
        self._close_log()

    def _open_log(self) -> None:
        self._close_log()
        log_dir = Path(__file__).resolve().parent / "pid_logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        path = log_dir / f"pid_{datetime.now():%Y%m%d_%H%M%S}.csv"
        self.log_file = path.open("w", newline="", encoding="utf-8-sig")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            "host_time", "board_ms", "left_target_mmps", "left_feedback_mmps",
            "left_pwm", "right_target_mmps", "right_feedback_mmps", "right_pwm",
            "left_count_5ms", "right_count_5ms",
        ])
        self.status_var.set(f"日志：{path}")

    def _close_log(self) -> None:
        if self.log_file is not None:
            self.log_file.close()
        self.log_file = None
        self.log_writer = None

    @staticmethod
    def _int(var: tk.StringVar, name: str, lower: int, upper: int) -> int:
        value = int(var.get().strip())
        if not lower <= value <= upper:
            raise ValueError(f"{name} 必须在 {lower}~{upper} 之间")
        return value

    def apply_params(self) -> None:
        try:
            kp_l = self._int(self.left_kp, "左 Kp", 0, 5000)
            kp_r = self._int(self.right_kp, "右 Kp", 0, 5000)
            ki_l = self._int(self.left_ki, "左 Ki", 0, 5000)
            ki_r = self._int(self.right_ki, "右 Ki", 0, 5000)
            limit_l = self._int(self.left_limit, "左限幅", 0, 3200)
            limit_r = self._int(self.right_limit, "右限幅", 0, 3200)
            self.serial.send(f"KP {kp_l} {kp_r}")
            self.serial.send(f"KI {ki_l} {ki_r}")
            self.serial.send(f"LIMIT {limit_l} {limit_r}")
            self.serial.send("RESET")
        except Exception as exc:
            messagebox.showerror("参数错误", str(exc))

    def arm(self) -> None:
        if not messagebox.askyesno(
            "安全确认",
            "确认车辆固定可靠、四个方向都不会碰到人或物，并且车轮已经完全架空？",
        ):
            return
        try:
            self.serial.send("ARM LIFTED")
        except Exception as exc:
            messagebox.showerror("ARM 失败", str(exc))

    def send_target(self) -> None:
        try:
            left = self._int(self.left_target, "左目标", -350, 350)
            right = self._int(self.right_target, "右目标", -350, 350)
            if not self.armed:
                raise RuntimeError("尚未 ARM；请先确认车轮架空")
            self.serial.send(f"TARGET {left} {right}")
        except Exception as exc:
            messagebox.showerror("目标错误", str(exc))

    def emergency_stop(self) -> None:
        self.autotune_cancel.set()
        try:
            if self.serial.connected:
                self.serial.send("STOP")
        except Exception:
            pass
        self.armed = False
        self.status_var.set("已发送紧急停止。")

    def start_autotune(self) -> None:
        if self.autotune_active:
            return
        if not self.serial.connected:
            messagebox.showerror("自动整定", "请先连接串口")
            return
        if not messagebox.askyesno(
            "自动整定安全确认",
            "自动整定会反复改变 PI 参数并运行单个轮子。确认车轮完全架空并可随时按紧急停止？",
        ):
            return
        try:
            side = self.auto_side.get()
            config = AutoTuneConfig(
                side=side,
                target=self._int(
                    self.left_target if side == "L" else self.right_target,
                    "整定目标",
                    50,
                    150,
                ),
                left_kp=self._int(self.left_kp, "左 Kp", 0, 5000),
                right_kp=self._int(self.right_kp, "右 Kp", 0, 5000),
                left_ki=self._int(self.left_ki, "左 Ki", 0, 5000),
                right_ki=self._int(self.right_ki, "右 Ki", 0, 5000),
                left_limit=self._int(self.left_limit, "左限幅", 0, 3200),
                right_limit=self._int(self.right_limit, "右限幅", 0, 3200),
            )
            selected_limit = config.left_limit if side == "L" else config.right_limit
            if selected_limit < 200:
                raise ValueError("自动整定轮的 PWM 限幅至少应为 200")
        except Exception as exc:
            messagebox.showerror("自动整定", str(exc))
            return
        self.autotune_cancel.clear()
        self.autotune_active = True
        threading.Thread(target=self._autotune_loop, args=(config,), daemon=True).start()

    def cancel_autotune(self) -> None:
        self.autotune_cancel.set()
        self.emergency_stop()

    def _snapshot_since(self, start: float) -> list[Sample]:
        with self.samples_lock:
            return [sample for sample in self.samples if sample.host_time >= start]

    @staticmethod
    def _score_trial(samples: list[Sample], side: str, target: int, limit: int) -> tuple[float, dict[str, float]]:
        if len(samples) < 15:
            return 1e9, {"reason": 1.0}
        raw_feedback = [s.left_feedback if side == "L" else s.right_feedback for s in samples]
        output = [s.left_output if side == "L" else s.right_output for s in samples]
        # 20 ms 遥测中的单点编码器速度存在明显量化跳变；用 100 ms 左右的
        # 移动平均评分，避免把正常的 0/1 个计数跳动误判为闭环振荡。
        window = min(5, len(raw_feedback))
        feedback = [
            sum(raw_feedback[max(0, i - window + 1):i + 1]) /
            len(raw_feedback[max(0, i - window + 1):i + 1])
            for i in range(len(raw_feedback))
        ]
        target_abs = max(abs(target), 1)
        errors = [target - value for value in feedback]
        normalized = [abs(error) / target_abs for error in errors]
        tail = max(5, len(samples) // 4)
        steady_error = sum(normalized[-tail:]) / tail
        overshoot = max(0.0, (max(abs(value) for value in feedback) - target_abs) / target_abs)
        saturation = sum(abs(value) >= int(limit * 0.95) for value in output) / len(output)
        sign_changes = 0
        previous = 0
        threshold = target_abs * 0.05
        for error in errors:
            sign = 1 if error > threshold else -1 if error < -threshold else 0
            if sign and previous and sign != previous:
                sign_changes += 1
            if sign:
                previous = sign
        mean_error = sum(normalized) / len(normalized)
        score = mean_error + 2.2 * overshoot + 1.6 * steady_error + 0.05 * sign_changes + 2.0 * saturation
        return score, {
            "mean": mean_error,
            "steady": steady_error,
            "overshoot": overshoot,
            "saturation": saturation,
            "changes": float(sign_changes),
        }

    def _autotune_loop(self, config: AutoTuneConfig) -> None:
        side = config.side
        target = config.target
        try:
            kp = config.left_kp if side == "L" else config.right_kp
            ki = config.left_ki if side == "L" else config.right_ki
            limit = min(1500, config.left_limit if side == "L" else config.right_limit)
            kp = min(max(kp, 20), 5000)
            ki = min(max(ki, 10), 5000)
            best_score = float("inf")
            best = (kp, ki)
            self.serial.send("STREAM 1")

            for iteration in range(1, AUTO_MAX_ITERATIONS + 1):
                if self.autotune_cancel.is_set():
                    break
                self._ui_status(f"自动整定 {side} 第 {iteration} 轮：Kp={kp}, Ki={ki}, limit={limit}")
                self.serial.send("STOP")
                if self.autotune_cancel.wait(0.20):
                    break
                self.serial.send("ARM LIFTED")
                if side == "L":
                    self.serial.send(f"KP {kp} {config.right_kp}")
                    self.serial.send(f"KI {ki} {config.right_ki}")
                    self.serial.send(f"LIMIT {limit} {config.right_limit}")
                    target_command = f"TARGET {target} 0"
                else:
                    self.serial.send(f"KP {config.left_kp} {kp}")
                    self.serial.send(f"KI {config.left_ki} {ki}")
                    self.serial.send(f"LIMIT {config.left_limit} {limit}")
                    target_command = f"TARGET 0 {target}"
                self.serial.send("RESET")
                start = time.monotonic()
                self.serial.send(target_command)
                while time.monotonic() - start < AUTO_TRIAL_SECONDS:
                    if self.autotune_cancel.wait(0.05):
                        break
                self.serial.send("TARGET 0 0")
                time.sleep(0.10)
                samples = self._snapshot_since(start + AUTO_SETTLE_SECONDS)
                score, metrics = self._score_trial(samples, side, target, limit)
                self._ui_log(
                    f"AUTO {side} iter={iteration} kp={kp} ki={ki} score={score:.3f} "
                    f"steady={metrics.get('steady', 9):.2f} over={metrics.get('overshoot', 9):.2f} "
                    f"sat={metrics.get('saturation', 9):.2f}"
                )
                if score < best_score:
                    best_score = score
                    best = (kp, ki)

                if metrics.get("saturation", 0.0) > 0.45 and metrics.get("steady", 0.0) > 0.25:
                    self._ui_status("自动整定停止：当前目标在此 PWM 限幅下可能无法达到，请检查负载或降低目标。")
                    break
                if metrics.get("overshoot", 0.0) > 0.20 or metrics.get("changes", 0.0) >= 6:
                    kp = max(20, int(kp * 0.80))
                    ki = max(10, int(ki * 0.75))
                elif metrics.get("steady", 1.0) > 0.18:
                    kp = min(5000, int(kp * 1.10) + 1)
                    ki = min(5000, int(ki * 1.25) + 1)
                elif metrics.get("steady", 1.0) > 0.08:
                    ki = min(5000, int(ki * 1.15) + 1)
                elif metrics.get("overshoot", 1.0) < 0.10 and metrics.get("steady", 1.0) <= 0.08:
                    self._ui_status(f"自动整定达到停止条件：{side} Kp={kp}, Ki={ki}")
                    break
                else:
                    kp = max(20, int(kp * 0.92))

            self.serial.send("STOP")
            if self.autotune_cancel.is_set():
                self._restore_autotune_config(config)
                self._ui_status("自动整定已取消，电机已停止并恢复整定前的 PI 参数。")
                return

            kp, ki = best
            if side == "L":
                self.serial.send(f"KP {kp} {config.right_kp}")
                self.serial.send(f"KI {ki} {config.right_ki}")
                self.serial.send(f"LIMIT {limit} {config.right_limit}")
            else:
                self.serial.send(f"KP {config.left_kp} {kp}")
                self.serial.send(f"KI {config.left_ki} {ki}")
                self.serial.send(f"LIMIT {config.left_limit} {limit}")
            self.serial.send("RESET")
            self._set_side_params(side, kp, ki, limit)
            self._ui_status(
                f"自动整定结束，最佳值已写回板端：{side} Kp={kp}, "
                f"Ki={ki}, score={best_score:.3f}。电机保持停止。"
            )
        except Exception as exc:
            self._ui_status(f"自动整定异常并停止：{exc}")
            try:
                self.serial.send("STOP")
                self._restore_autotune_config(config)
            except Exception:
                pass
        finally:
            self.autotune_active = False
            self.armed = False

    def _restore_autotune_config(self, config: AutoTuneConfig) -> None:
        self.serial.send(f"KP {config.left_kp} {config.right_kp}")
        self.serial.send(f"KI {config.left_ki} {config.right_ki}")
        self.serial.send(f"LIMIT {config.left_limit} {config.right_limit}")
        self.serial.send("RESET")
        self.ui_events.put((
            "all_params",
            (
                config.left_kp, config.right_kp,
                config.left_ki, config.right_ki,
                config.left_limit, config.right_limit,
            ),
        ))

    def _set_side_params(self, side: str, kp: int, ki: int, limit: int) -> None:
        self.ui_events.put(("side_params", (side, kp, ki, limit)))

    def _ui_status(self, text: str) -> None:
        self.ui_events.put(("status", text))

    def _ui_log(self, text: str) -> None:
        self.ui_events.put(("log", text))

    def _drain_ui_events(self) -> None:
        try:
            while True:
                event, payload = self.ui_events.get_nowait()
                if event == "status":
                    self.status_var.set(str(payload))
                elif event == "log":
                    self._append_console(str(payload))
                elif event == "side_params":
                    side, kp, ki, limit = payload
                    if side == "L":
                        self.left_kp.set(str(kp))
                        self.left_ki.set(str(ki))
                        self.left_limit.set(str(limit))
                    else:
                        self.right_kp.set(str(kp))
                        self.right_ki.set(str(ki))
                        self.right_limit.set(str(limit))
                elif event == "all_params":
                    kp_l, kp_r, ki_l, ki_r, limit_l, limit_r = payload
                    self.left_kp.set(str(kp_l))
                    self.right_kp.set(str(kp_r))
                    self.left_ki.set(str(ki_l))
                    self.right_ki.set(str(ki_r))
                    self.left_limit.set(str(limit_l))
                    self.right_limit.set(str(limit_r))
        except queue.Empty:
            pass

    def _append_console(self, text: str) -> None:
        self.console.configure(state="normal")
        self.console.insert("end", text + "\n")
        self.console.see("end")
        self.console.configure(state="disabled")

    def handle_line(self, line: str) -> None:
        if not line:
            return
        parts = line.split(",")
        if parts[0] == "T" and len(parts) == 10:
            try:
                values = [int(value) for value in parts[1:]]
                sample = Sample(time.monotonic(), *values)
            except ValueError:
                return
            with self.samples_lock:
                self.samples.append(sample)
            if self.log_writer is not None:
                self.log_writer.writerow([sample.host_time, *values])
                self.log_file.flush()
            return
        if parts[0] == "P" and len(parts) == 8:
            try:
                self.armed = bool(int(parts[1]))
                self.left_kp.set(parts[2])
                self.left_ki.set(parts[3])
                self.left_limit.set(parts[4])
                self.right_kp.set(parts[5])
                self.right_ki.set(parts[6])
                self.right_limit.set(parts[7])
            except ValueError:
                pass
        elif line == "OK,ARMED":
            self.armed = True
            self.status_var.set("已 ARM，等待目标；上位机失联 1 秒会自动停车。")
        elif line.startswith("OK,STOPPED") or line.startswith("EVENT,TIMEOUT_STOP"):
            self.armed = False
            self.status_var.set("电机已停止。")
        elif line.startswith("LOCAL"):
            self.armed = False
            self.connection_var.set("串口异常断开")
            self.status_var.set(line)
        elif line.startswith("ERR"):
            self.status_var.set(line)
        self._append_console(line)

    def poll(self) -> None:
        self._drain_ui_events()
        try:
            while True:
                self.handle_line(self.lines.get_nowait())
        except queue.Empty:
            pass

        now = time.monotonic()
        if self.serial.connected and now - self.last_keepalive >= KEEPALIVE_SECONDS:
            try:
                self.serial.send("KEEP")
            except Exception as exc:
                self.status_var.set(f"KEEP 失败：{exc}")
            self.last_keepalive = now

        with self.samples_lock:
            plot_samples = list(self.samples)
        try:
            limit = max(int(self.left_limit.get()), int(self.right_limit.get()))
        except ValueError:
            limit = 3200
        self.speed_plot.draw(plot_samples, "speed", limit)
        self.pwm_plot.draw(plot_samples, "pwm", limit)
        self.root.after(100, self.poll)

    def on_close(self) -> None:
        self.autotune_cancel.set()
        self.serial.close(send_stop=True)
        self._close_log()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="G3507 CH340 wheel-speed PI tuner")
    parser.add_argument("--port", help="串口，例如 COM6")
    args = parser.parse_args()
    root = tk.Tk()
    PIDTunerApp(root, args.port)
    root.mainloop()


if __name__ == "__main__":
    main()
