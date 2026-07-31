#!/usr/bin/env python3
"""UART3 wireless-DAP line-tracking monitor with live plots and CSV export."""

from __future__ import annotations

import argparse
import csv
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Callable

import serial
from serial.tools import list_ports

from track_telemetry_protocol import (
    CSV_COLUMNS,
    EVENT_TEXT,
    RESULT_TEXT,
    STATE_TEXT,
    TelemetryParser,
    TelemetrySample,
    sample_to_csv_row,
)


DEFAULT_BAUD_RATE = 115200
PLOT_WINDOW_SECONDS = 10.0
ROLLING_SAMPLE_LIMIT = 2000
SESSION_SAMPLE_LIMIT = 180000  # 100 Hz at 30 minutes; protects against accidental endless logging.
ACTIVE_STATES = {1, 2}


class SerialWorker:
    """Read-only serial worker; this program never writes a byte back to the MCU."""

    def __init__(self, events: queue.Queue[tuple[str, object]]) -> None:
        self.events = events
        self.port: serial.Serial | None = None
        self.reader: threading.Thread | None = None
        self.stop_event = threading.Event()

    @property
    def connected(self) -> bool:
        port = self.port
        return port is not None and port.is_open

    def connect(self, port_name: str, baud_rate: int) -> None:
        self.close()
        port = serial.Serial(port_name, baud_rate, timeout=0.10)
        port.reset_input_buffer()
        stop_event = threading.Event()
        parser = TelemetryParser()
        self.port = port
        self.stop_event = stop_event
        self.reader = threading.Thread(
            target=self._read_loop,
            args=(port, stop_event, parser),
            daemon=True,
        )
        self.reader.start()
        self.events.put(("connected", (port_name, baud_rate)))

    def _read_loop(
        self,
        port: serial.Serial,
        stop_event: threading.Event,
        parser: TelemetryParser,
    ) -> None:
        last_stats = (-1, -1, -1, -1)
        try:
            while not stop_event.is_set():
                waiting = port.in_waiting
                raw = port.read(min(max(waiting, 1), 4096))
                if raw:
                    for sample in parser.feed(raw):
                        self.events.put(("sample", sample))
                stats = (
                    parser.valid_frames,
                    parser.crc_errors,
                    parser.format_errors,
                    parser.discarded_bytes,
                )
                if stats != last_stats:
                    self.events.put(("parser_stats", stats))
                    last_stats = stats
        except (serial.SerialException, OSError) as exc:
            if not stop_event.is_set():
                self.events.put(("serial_error", str(exc)))
        finally:
            try:
                port.close()
            except Exception:
                pass
            if self.port is port:
                self.port = None
            self.events.put(("disconnected", None))

    def close(self) -> None:
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
            reader.join(timeout=0.35)


@dataclass(frozen=True)
class PlotSeries:
    label: str
    color: str
    value: Callable[[TelemetrySample], float]


class TimeSeriesCanvas(tk.Canvas):
    """Small dependency-free live plot suitable for the bundled Python runtime."""

    def __init__(
        self,
        master: tk.Misc,
        title: str,
        series: list[PlotSeries],
        unit: str,
        **kwargs: object,
    ) -> None:
        super().__init__(
            master,
            background="#10151c",
            highlightthickness=1,
            highlightbackground="#2f3b49",
            **kwargs,
        )
        self.title = title
        self.series = series
        self.unit = unit

    def draw(self, samples: list[TelemetrySample]) -> None:
        self.delete("all")
        width = max(self.winfo_width(), 500)
        height = max(self.winfo_height(), 135)
        left, top, right, bottom = 58, 28, width - 12, height - 25
        self.create_text(10, 7, anchor="nw", text=self.title, fill="#e7edf5")

        legend_x = 170
        for series in self.series:
            self.create_line(legend_x, 14, legend_x + 18, 14, fill=series.color, width=2)
            self.create_text(
                legend_x + 22,
                7,
                anchor="nw",
                text=series.label,
                fill=series.color,
            )
            legend_x += 92

        self.create_rectangle(left, top, right, bottom, outline="#44515f")
        if not samples:
            self.create_text(
                (left + right) / 2,
                (top + bottom) / 2,
                text="等待 UART3 遥测数据",
                fill="#8090a0",
            )
            return

        latest_time = samples[-1].host_monotonic
        earliest_time = latest_time - PLOT_WINDOW_SECONDS
        visible = [sample for sample in samples if sample.host_monotonic >= earliest_time]
        if not visible:
            return

        all_values = [series.value(sample) for sample in visible for series in self.series]
        minimum = min(min(all_values), 0.0)
        maximum = max(max(all_values), 0.0)
        if maximum - minimum < 1e-6:
            maximum += 1.0
            minimum -= 1.0
        padding = (maximum - minimum) * 0.10
        maximum += padding
        minimum -= padding

        for grid_index in range(5):
            ratio = grid_index / 4
            y = top + ratio * (bottom - top)
            value = maximum - ratio * (maximum - minimum)
            self.create_line(left, y, right, y, fill="#26313d")
            self.create_text(
                left - 5,
                y,
                anchor="e",
                text=f"{value:.2f}",
                fill="#8090a0",
            )
        self.create_text(
            8,
            top + 2,
            anchor="nw",
            text=self.unit,
            fill="#8090a0",
        )

        point_budget = max(int(right - left), 1)
        step = max(1, len(visible) // point_budget)
        reduced = visible[::step]
        if reduced[-1] is not visible[-1]:
            reduced.append(visible[-1])

        time_span = max(latest_time - earliest_time, 0.001)
        value_span = maximum - minimum
        for series in self.series:
            coordinates: list[float] = []
            for sample in reduced:
                x = left + (sample.host_monotonic - earliest_time) / time_span * (right - left)
                value = series.value(sample)
                y = bottom - (value - minimum) / value_span * (bottom - top)
                coordinates.extend((x, y))
            if len(coordinates) >= 4:
                self.create_line(*coordinates, fill=series.color, width=2, smooth=False)


class SensorCanvas(tk.Canvas):
    """Display CH1..CH8 in physical left-to-right order; black=detected line."""

    def __init__(self, master: tk.Misc, **kwargs: object) -> None:
        super().__init__(master, height=116, background="#f4f6f8", **kwargs)

    def draw(self, sample: TelemetrySample | None) -> None:
        self.delete("all")
        width = max(self.winfo_width(), 340)
        gap = 6
        cell_width = (width - 18 - gap * 7) / 8
        black_channels = sample.black_channels() if sample else (0,) * 8
        raw_channels = sample.raw_channels() if sample else (1,) * 8

        self.create_text(8, 6, anchor="nw", text="八路探头（左 CH1 → 右 CH8）", fill="#26313d")
        for index in range(8):
            x0 = 8 + index * (cell_width + gap)
            x1 = x0 + cell_width
            detected = bool(black_channels[index])
            fill = "#111111" if detected else "#ffffff"
            text_color = "#ffffff" if detected else "#202833"
            outline = "#d63b3b" if sample and sample.marker else "#526170"
            self.create_rectangle(x0, 32, x1, 82, fill=fill, outline=outline, width=2)
            self.create_text(
                (x0 + x1) / 2,
                50,
                text=f"CH{index + 1}",
                fill=text_color,
            )
            self.create_text(
                (x0 + x1) / 2,
                69,
                text=f"R{raw_channels[index]}",
                fill=text_color,
            )
        marker_text = "A点横线候选" if sample and sample.marker else "普通循迹线"
        active = sample.active_count if sample else 0
        self.create_text(
            8,
            94,
            anchor="nw",
            text=f"黑色通道数={active}   {marker_text}",
            fill="#c62828" if sample and sample.marker else "#415466",
        )


class TrackMonitorApp:
    def __init__(self, root: tk.Tk, initial_port: str | None, baud_rate: int) -> None:
        self.root = root
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.serial = SerialWorker(self.events)
        self.rolling_samples: deque[TelemetrySample] = deque(maxlen=ROLLING_SAMPLE_LIMIT)
        self.session_samples: list[TelemetrySample] = []
        self.latest_sample: TelemetrySample | None = None
        self.last_race_state: int | None = None
        self.recording = False
        self.capture_end_reason = ""
        self.total_packet_gaps = 0
        self.parser_stats = (0, 0, 0, 0)
        self.port_map: dict[str, str] = {}
        self.last_plot_refresh = 0.0

        self.port_var = tk.StringVar(value=initial_port or "")
        self.baud_var = tk.StringVar(value=str(baud_rate))
        self.connection_var = tk.StringVar(value="未连接")
        self.capture_var = tk.StringVar(value="未记录")
        self.stats_var = tk.StringVar(value="有效帧 0 | CRC错误 0 | 序号缺口 0")
        self.auto_capture_var = tk.BooleanVar(value=True)

        self.root.title("G3507 UART3 循迹实时监视器")
        self.root.geometry("1420x900")
        self.root.minsize(1100, 720)
        self._build_ui()
        self.refresh_ports(preferred=initial_port)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(30, self.poll)

    def _build_ui(self) -> None:
        controls = ttk.Frame(self.root, padding=(8, 7))
        controls.pack(fill="x")
        ttk.Label(controls, text="无线 DAP 串口").pack(side="left")
        self.port_box = ttk.Combobox(
            controls, textvariable=self.port_var, width=42, state="readonly"
        )
        self.port_box.pack(side="left", padx=(6, 4))
        ttk.Button(controls, text="刷新", command=self.refresh_ports).pack(side="left")
        ttk.Label(controls, text="波特率").pack(side="left", padx=(12, 4))
        ttk.Combobox(
            controls,
            textvariable=self.baud_var,
            values=("115200",),
            width=9,
            state="readonly",
        ).pack(side="left")
        ttk.Button(controls, text="连接", command=self.connect).pack(side="left", padx=(8, 3))
        ttk.Button(controls, text="断开", command=self.disconnect).pack(side="left")
        ttk.Checkbutton(
            controls,
            text="随比赛自动记录",
            variable=self.auto_capture_var,
        ).pack(side="left", padx=(16, 4))
        ttk.Button(controls, text="手动开始记录", command=self.start_manual_capture).pack(
            side="left", padx=3
        )
        ttk.Button(controls, text="停止记录", command=self.stop_manual_capture).pack(
            side="left", padx=3
        )
        ttk.Button(controls, text="导出 CSV", command=self.export_csv).pack(
            side="left", padx=(10, 3)
        )
        ttk.Button(controls, text="清空记录", command=self.clear_capture).pack(side="left")

        summary = ttk.Frame(self.root, padding=(8, 0, 8, 6))
        summary.pack(fill="x")
        ttk.Label(summary, textvariable=self.connection_var).pack(side="left")
        ttk.Label(summary, text="   ").pack(side="left")
        ttk.Label(summary, textvariable=self.capture_var).pack(side="left")
        ttk.Label(summary, textvariable=self.stats_var).pack(side="right")

        body = ttk.Panedwindow(self.root, orient="horizontal")
        body.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        left = ttk.Frame(body, padding=6)
        right = ttk.Frame(body, padding=(6, 0, 0, 0))
        body.add(left, weight=1)
        body.add(right, weight=3)

        self.sensor_canvas = SensorCanvas(left, highlightthickness=1)
        self.sensor_canvas.pack(fill="x", pady=(0, 6))
        self.sensor_canvas.draw(None)

        self.value_tree = ttk.Treeview(
            left,
            columns=("item", "value"),
            show="headings",
            height=28,
        )
        self.value_tree.heading("item", text="关键变量")
        self.value_tree.heading("value", text="当前值")
        self.value_tree.column("item", width=165, anchor="w")
        self.value_tree.column("value", width=175, anchor="e")
        self.value_tree.pack(fill="both", expand=True)
        self.value_items: dict[str, str] = {}
        for key, label in [
            ("state", "比赛状态"),
            ("elapsed", "运行时间"),
            ("event", "最近事件"),
            ("mask", "原始/黑线位图"),
            ("raw_error", "原始误差"),
            ("steering_error", "控制采用误差"),
            ("filtered_error", "源PD误差(兼容字段)"),
            ("derivative", "离散微分"),
            ("p_term", "P项"),
            ("d_term", "D项"),
            ("correction", "差速修正"),
            ("base", "基础速度 请求/期望/实际"),
            ("left_speed", "左轮 目标/反馈"),
            ("right_speed", "右轮 目标/反馈"),
            ("encoder", "编码器 左/右"),
            ("pwm", "PWM 左/右"),
            ("line_lost", "丢线样本/时间"),
            ("center", "中心稳定样本"),
            ("marker", "A点 清除/确认"),
            ("flags", "就绪/使能/恢复/故障"),
            ("sequence", "帧序号/本帧缺口"),
            ("mcu_drop", "MCU遥测丢帧累计"),
            ("board", "板端ms/控制周期"),
        ]:
            item = self.value_tree.insert("", "end", values=(label, "--"))
            self.value_items[key] = item

        plots = [
            TimeSeriesCanvas(
                right,
                "源循迹误差与离散微分",
                [
                    PlotSeries("原始", "#ff6b6b", lambda s: s.raw_error),
                    PlotSeries("采用", "#ffd166", lambda s: s.steering_error),
                    PlotSeries("源PD", "#4dd0e1", lambda s: s.filtered_error),
                    PlotSeries("微分", "#b388ff", lambda s: s.filtered_derivative),
                ],
                "error",
                height=170,
            ),
            TimeSeriesCanvas(
                right,
                "转向修正与动态基础速度",
                [
                    PlotSeries("P项", "#ff8a65", lambda s: s.p_term_mps),
                    PlotSeries("D项", "#ba68c8", lambda s: s.d_term_mps),
                    PlotSeries("修正", "#29b6f6", lambda s: s.correction_mps),
                    PlotSeries("基础", "#66bb6a", lambda s: s.dynamic_base_mps),
                ],
                "m/s",
                height=170,
            ),
            TimeSeriesCanvas(
                right,
                "左右轮目标与反馈",
                [
                    PlotSeries("左目标", "#42a5f5", lambda s: s.left_target_mps),
                    PlotSeries("左反馈", "#90caf9", lambda s: s.left_speed_mps),
                    PlotSeries("右目标", "#ef5350", lambda s: s.right_target_mps),
                    PlotSeries("右反馈", "#ef9a9a", lambda s: s.right_speed_mps),
                ],
                "m/s",
                height=170,
            ),
            TimeSeriesCanvas(
                right,
                "电机 PWM 输出",
                [
                    PlotSeries("左PWM", "#42a5f5", lambda s: float(s.left_pwm)),
                    PlotSeries("右PWM", "#ef5350", lambda s: float(s.right_pwm)),
                ],
                "count",
                height=150,
            ),
        ]
        self.plots = plots
        for plot in plots:
            plot.pack(fill="both", expand=True, pady=(0, 5))
            plot.draw([])

    def refresh_ports(self, preferred: str | None = None) -> None:
        current = preferred or self.port_var.get()
        self.port_map.clear()
        labels: list[str] = []
        for port in sorted(list_ports.comports(), key=lambda item: item.device):
            label = f"{port.device} - {port.description}"
            labels.append(label)
            self.port_map[label] = port.device
        self.port_box["values"] = labels
        selected = ""
        for label, device in self.port_map.items():
            if current in (label, device):
                selected = label
                break
        if not selected and labels:
            selected = labels[0]
        self.port_var.set(selected)

    def connect(self) -> None:
        label = self.port_var.get()
        port_name = self.port_map.get(label, label.split(" ", 1)[0] if label else "")
        if not port_name:
            messagebox.showerror("串口", "没有选择无线 DAP 串口。")
            return
        try:
            baud = int(self.baud_var.get())
            self.serial.connect(port_name, baud)
            self.rolling_samples.clear()
            self.latest_sample = None
            self.last_race_state = None
            self.total_packet_gaps = 0
            self.parser_stats = (0, 0, 0, 0)
        except (ValueError, serial.SerialException, OSError) as exc:
            messagebox.showerror("连接失败", str(exc))

    def disconnect(self) -> None:
        self.serial.close()
        self.connection_var.set("未连接")
        if self.recording:
            self._finish_capture("串口断开")

    def start_manual_capture(self) -> None:
        self._begin_capture("电脑端手动开始")

    def stop_manual_capture(self) -> None:
        if self.recording:
            self._finish_capture("电脑端手动停止记录")

    def clear_capture(self) -> None:
        if self.recording:
            messagebox.showwarning("正在记录", "请先停止当前记录。")
            return
        self.session_samples.clear()
        self.capture_end_reason = ""
        self.capture_var.set("未记录")

    def _begin_capture(self, source: str) -> None:
        self.session_samples.clear()
        self.capture_end_reason = ""
        self.recording = True
        self.capture_var.set(f"正在记录：{source}")

    def _finish_capture(self, reason: str) -> None:
        self.recording = False
        self.capture_end_reason = reason
        self.capture_var.set(
            f"记录完成：{reason}，{len(self.session_samples)} 个样本"
        )

    def _handle_sample(self, sample: TelemetrySample) -> None:
        previous_state = self.last_race_state
        previous_active = previous_state in ACTIVE_STATES
        current_active = sample.state in ACTIVE_STATES

        if self.auto_capture_var.get() and not self.recording:
            if sample.event == 1 or (not previous_active and current_active):
                self._begin_capture("检测到 PB21 启动")

        if self.recording:
            if len(self.session_samples) >= SESSION_SAMPLE_LIMIT:
                self._finish_capture("达到30分钟安全上限")
            else:
                self.session_samples.append(sample)

        if self.recording:
            if sample.event == 3 or (previous_active and sample.state == 0):
                self._finish_capture("PB21人工停止")
            elif sample.event == 2 or (previous_active and sample.state == 3):
                reason = RESULT_TEXT.get(sample.result, f"结果{sample.result}")
                self._finish_capture(reason)

        self.last_race_state = sample.state
        self.latest_sample = sample
        self.rolling_samples.append(sample)
        self.total_packet_gaps += sample.packet_gap

    def export_csv(self) -> None:
        if not self.session_samples:
            messagebox.showwarning("没有数据", "当前没有可导出的整圈或手动记录数据。")
            return

        output_dir = Path.home() / "Documents" / "G3507_Telemetry"
        output_dir.mkdir(parents=True, exist_ok=True)
        reason = self.capture_end_reason or "recording"
        safe_reason = "".join(character for character in reason if character.isalnum()) or "data"
        default_name = (
            f"track_telemetry_{datetime.now():%Y%m%d_%H%M%S}_{safe_reason}.csv"
        )
        path = filedialog.asksaveasfilename(
            title="导出循迹遥测 CSV",
            initialdir=output_dir,
            initialfile=default_name,
            defaultextension=".csv",
            filetypes=(("CSV 文件", "*.csv"), ("所有文件", "*.*")),
        )
        if not path:
            return

        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as file:
                writer = csv.writer(file)
                writer.writerow(CSV_COLUMNS)
                for sample in self.session_samples:
                    writer.writerow(sample_to_csv_row(sample, self.capture_end_reason))
        except OSError as exc:
            messagebox.showerror("导出失败", str(exc))
            return
        messagebox.showinfo(
            "导出完成",
            f"已写入 {len(self.session_samples)} 个样本：\n{path}",
        )

    def _update_current_values(self, sample: TelemetrySample) -> None:
        values = {
            "state": f"{sample.state_text} ({sample.state}) / {sample.result_text}",
            "elapsed": f"{sample.elapsed_ms / 1000.0:.3f} s",
            "event": sample.event_text,
            "mask": f"0x{sample.raw_track_mask:02X} / 0x{sample.track_mask:02X}",
            "raw_error": f"{sample.raw_error:+.2f}",
            "steering_error": f"{sample.steering_error:+.2f}",
            "filtered_error": f"{sample.filtered_error:+.2f}",
            "derivative": f"{sample.filtered_derivative:+.2f}",
            "p_term": f"{sample.p_term_mps:+.4f} m/s",
            "d_term": f"{sample.d_term_mps:+.4f} m/s",
            "correction": f"{sample.correction_mps:+.4f} m/s",
            "base": (
                f"{sample.requested_base_mps:.3f} / "
                f"{sample.desired_base_mps:.3f} / {sample.dynamic_base_mps:.3f}"
            ),
            "left_speed": f"{sample.left_target_mps:+.3f} / {sample.left_speed_mps:+.3f}",
            "right_speed": f"{sample.right_target_mps:+.3f} / {sample.right_speed_mps:+.3f}",
            "encoder": f"{sample.left_encoder_count} / {sample.right_encoder_count}",
            "pwm": f"{sample.left_pwm} / {sample.right_pwm}",
            "line_lost": f"{sample.line_lost_samples} / {sample.line_lost_ms} ms",
            "center": str(sample.center_confirm_samples),
            "marker": f"{sample.marker_clear_streak} / {sample.marker_confirm_streak}",
            "flags": (
                f"{int(sample.track_ready)}/{int(sample.armed)}/"
                f"{int(sample.recovering)}/{int(sample.line_fault)}"
            ),
            "sequence": f"{sample.sequence} / +{sample.packet_gap}",
            "mcu_drop": str(sample.mcu_dropped_frames),
            "board": f"{sample.board_ms} / {sample.control_tick}",
        }
        for key, value in values.items():
            self.value_tree.set(self.value_items[key], "value", value)
        self.sensor_canvas.draw(sample)

    def _update_stats_text(self) -> None:
        valid, crc_errors, format_errors, discarded = self.parser_stats
        mcu_drop = self.latest_sample.mcu_dropped_frames if self.latest_sample else 0
        self.stats_var.set(
            f"有效帧 {valid} | CRC错误 {crc_errors} | 格式错误 {format_errors} | "
            f"序号缺口 {self.total_packet_gaps} | MCU丢帧 {mcu_drop} | 丢弃字节 {discarded}"
        )

    def poll(self) -> None:
        latest_changed = False
        try:
            while True:
                event, payload = self.events.get_nowait()
                if event == "sample":
                    self._handle_sample(payload)  # type: ignore[arg-type]
                    latest_changed = True
                elif event == "parser_stats":
                    self.parser_stats = payload  # type: ignore[assignment]
                elif event == "connected":
                    port_name, baud = payload  # type: ignore[misc]
                    self.connection_var.set(f"已连接 {port_name} @ {baud}，只读接收")
                elif event == "serial_error":
                    self.connection_var.set(f"串口异常：{payload}")
                    if self.recording:
                        self._finish_capture("串口异常断开")
                elif event == "disconnected" and not self.serial.connected:
                    if not self.connection_var.get().startswith("串口异常"):
                        self.connection_var.set("未连接")
        except queue.Empty:
            pass

        if latest_changed and self.latest_sample is not None:
            self._update_current_values(self.latest_sample)
        self._update_stats_text()

        now = time.monotonic()
        if now - self.last_plot_refresh >= 0.10:
            samples = list(self.rolling_samples)
            for plot in self.plots:
                plot.draw(samples)
            self.last_plot_refresh = now
        self.root.after(30, self.poll)

    def on_close(self) -> None:
        self.serial.close()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="G3507 UART3 wireless-DAP line-tracking telemetry monitor"
    )
    parser.add_argument("--port", help="serial port, for example COM6")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD_RATE)
    args = parser.parse_args()

    root = tk.Tk()
    TrackMonitorApp(root, args.port, args.baud)
    root.mainloop()


if __name__ == "__main__":
    main()
