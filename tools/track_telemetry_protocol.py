#!/usr/bin/env python3
"""G3507 UART3 fixed-frame telemetry decoder shared by the GUI and tests."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import struct
import time
from typing import Iterable


SYNC = b"\xA5\x5A"
PROTOCOL_VERSION = 1
FRAME_TYPE_CONTROL = 1
PAYLOAD_LENGTH = 64
HEADER = struct.Struct("<2sBBHH")
PAYLOAD = struct.Struct("<III10BHHH18h")
CRC = struct.Struct("<H")
FRAME_LENGTH = HEADER.size + PAYLOAD_LENGTH + CRC.size

FLAG_TRACK_READY = 1 << 0
FLAG_ARMED = 1 << 1
FLAG_MANUAL_SPEED = 1 << 2
FLAG_MARKER = 1 << 3
FLAG_RECOVERING = 1 << 4
FLAG_LINE_FAULT = 1 << 5
FLAG_MARKER_CLEARED = 1 << 6
FLAG_APPROACH_ACTIVE = 1 << 7

STATE_TEXT = {0: "等待", 1: "驶离A点", 2: "正常跑圈", 3: "已停止"}
RESULT_TEXT = {0: "无", 1: "正常完成", 2: "持续丢线", 3: "启动拒绝"}
EVENT_TEXT = {0: "无", 1: "开始", 2: "结束", 3: "人工停止"}


def crc16_ccitt(data: bytes | bytearray | memoryview) -> int:
    """CRC16-CCITT-FALSE, matching APP/Telemetry_Stream.c exactly."""

    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class TelemetrySample:
    """One validated UART3 frame converted to engineering units."""

    host_monotonic: float
    host_time_iso: str
    sequence: int
    packet_gap: int
    board_ms: int
    control_tick: int
    elapsed_ms: int
    state: int
    result: int
    event: int
    raw_track_mask: int
    track_mask: int
    active_count: int
    marker_clear_streak: int
    marker_confirm_streak: int
    center_confirm_samples: int
    line_lost_samples: int
    flags: int
    line_lost_ms: int
    mcu_dropped_frames: int
    raw_error: float
    steering_error: float
    filtered_error: float
    filtered_derivative: float
    p_term_mps: float
    d_term_mps: float
    correction_mps: float
    requested_base_mps: float
    desired_base_mps: float
    dynamic_base_mps: float
    left_target_mps: float
    right_target_mps: float
    left_speed_mps: float
    right_speed_mps: float
    left_encoder_count: int
    right_encoder_count: int
    left_pwm: int
    right_pwm: int

    def flag(self, mask: int) -> bool:
        return bool(self.flags & mask)

    @property
    def track_ready(self) -> bool:
        return self.flag(FLAG_TRACK_READY)

    @property
    def armed(self) -> bool:
        return self.flag(FLAG_ARMED)

    @property
    def manual_speed_mode(self) -> bool:
        return self.flag(FLAG_MANUAL_SPEED)

    @property
    def marker(self) -> bool:
        return self.flag(FLAG_MARKER)

    @property
    def recovering(self) -> bool:
        return self.flag(FLAG_RECOVERING)

    @property
    def line_fault(self) -> bool:
        return self.flag(FLAG_LINE_FAULT)

    @property
    def marker_cleared(self) -> bool:
        return self.flag(FLAG_MARKER_CLEARED)

    @property
    def approach_active(self) -> bool:
        return self.flag(FLAG_APPROACH_ACTIVE)

    @property
    def state_text(self) -> str:
        return STATE_TEXT.get(self.state, f"未知({self.state})")

    @property
    def result_text(self) -> str:
        return RESULT_TEXT.get(self.result, f"未知({self.result})")

    @property
    def event_text(self) -> str:
        return EVENT_TEXT.get(self.event, f"未知({self.event})")

    def raw_channels(self) -> tuple[int, ...]:
        """CH1..CH8 raw GPIO levels: white=1, black=0."""

        return tuple(1 if self.raw_track_mask & (0x80 >> index) else 0 for index in range(8))

    def black_channels(self) -> tuple[int, ...]:
        """CH1..CH8 normalized line bits: black=1."""

        return tuple(1 if self.track_mask & (0x80 >> index) else 0 for index in range(8))


class TelemetryParser:
    """Incremental stream parser with CRC recovery and sequence-gap tracking."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.last_sequence: int | None = None
        self.valid_frames = 0
        self.crc_errors = 0
        self.format_errors = 0
        self.discarded_bytes = 0

    def reset(self) -> None:
        self.buffer.clear()
        self.last_sequence = None
        self.valid_frames = 0
        self.crc_errors = 0
        self.format_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[TelemetrySample]:
        if data:
            self.buffer.extend(data)
        samples: list[TelemetrySample] = []

        while True:
            sync_index = self.buffer.find(SYNC)
            if sync_index < 0:
                # Keep one trailing 0xA5 because it may be the first byte of the next sync.
                keep = 1 if self.buffer.endswith(SYNC[:1]) else 0
                discard = len(self.buffer) - keep
                if discard > 0:
                    del self.buffer[:discard]
                    self.discarded_bytes += discard
                break
            if sync_index > 0:
                del self.buffer[:sync_index]
                self.discarded_bytes += sync_index
            if len(self.buffer) < HEADER.size:
                break

            sync, version, frame_type, payload_length, sequence = HEADER.unpack_from(self.buffer)
            if (
                sync != SYNC
                or version != PROTOCOL_VERSION
                or frame_type != FRAME_TYPE_CONTROL
                or payload_length != PAYLOAD_LENGTH
            ):
                del self.buffer[0]
                self.format_errors += 1
                continue

            total_length = HEADER.size + payload_length + CRC.size
            if len(self.buffer) < total_length:
                break
            frame = bytes(self.buffer[:total_length])
            expected_crc = CRC.unpack_from(frame, total_length - CRC.size)[0]
            actual_crc = crc16_ccitt(frame[2 : total_length - CRC.size])
            if expected_crc != actual_crc:
                del self.buffer[0]
                self.crc_errors += 1
                continue

            values = PAYLOAD.unpack_from(frame, HEADER.size)
            gap = self._sequence_gap(sequence)
            samples.append(self._decode(sequence, gap, values))
            self.valid_frames += 1
            del self.buffer[:total_length]

        return samples

    def _sequence_gap(self, sequence: int) -> int:
        gap = 0
        if self.last_sequence is not None:
            delta = (sequence - self.last_sequence - 1) & 0xFFFF
            # A large backwards jump is treated as an MCU reboot/stream restart, not 65k lost frames.
            if delta < 0x8000:
                gap = delta
        self.last_sequence = sequence
        return gap

    @staticmethod
    def _decode(sequence: int, gap: int, values: tuple[int, ...]) -> TelemetrySample:
        (
            board_ms,
            control_tick,
            elapsed_ms,
            state,
            result,
            event,
            raw_track_mask,
            track_mask,
            active_count,
            marker_clear_streak,
            marker_confirm_streak,
            center_confirm_samples,
            line_lost_samples,
            flags,
            line_lost_ms,
            mcu_dropped_frames,
            raw_error,
            steering_error,
            filtered_error,
            filtered_derivative,
            p_term_mmps,
            d_term_mmps,
            correction_mmps,
            requested_base_mmps,
            desired_base_mmps,
            dynamic_base_mmps,
            left_target_mmps,
            right_target_mmps,
            left_speed_mmps,
            right_speed_mmps,
            left_encoder_count,
            right_encoder_count,
            left_pwm,
            right_pwm,
        ) = values
        return TelemetrySample(
            host_monotonic=time.monotonic(),
            host_time_iso=datetime.now().isoformat(timespec="milliseconds"),
            sequence=sequence,
            packet_gap=gap,
            board_ms=board_ms,
            control_tick=control_tick,
            elapsed_ms=elapsed_ms,
            state=state,
            result=result,
            event=event,
            raw_track_mask=raw_track_mask,
            track_mask=track_mask,
            active_count=active_count,
            marker_clear_streak=marker_clear_streak,
            marker_confirm_streak=marker_confirm_streak,
            center_confirm_samples=center_confirm_samples,
            line_lost_samples=line_lost_samples,
            flags=flags,
            line_lost_ms=line_lost_ms,
            mcu_dropped_frames=mcu_dropped_frames,
            raw_error=raw_error / 100.0,
            steering_error=steering_error / 100.0,
            filtered_error=filtered_error / 100.0,
            filtered_derivative=filtered_derivative / 100.0,
            p_term_mps=p_term_mmps / 1000.0,
            d_term_mps=d_term_mmps / 1000.0,
            correction_mps=correction_mmps / 1000.0,
            requested_base_mps=requested_base_mmps / 1000.0,
            desired_base_mps=desired_base_mmps / 1000.0,
            dynamic_base_mps=dynamic_base_mmps / 1000.0,
            left_target_mps=left_target_mmps / 1000.0,
            right_target_mps=right_target_mmps / 1000.0,
            left_speed_mps=left_speed_mmps / 1000.0,
            right_speed_mps=right_speed_mmps / 1000.0,
            left_encoder_count=left_encoder_count,
            right_encoder_count=right_encoder_count,
            left_pwm=left_pwm,
            right_pwm=right_pwm,
        )


CSV_COLUMNS = [
    "host_time",
    "sequence",
    "packet_gap_before",
    "board_ms",
    "control_tick",
    "elapsed_ms",
    "race_state",
    "race_state_text",
    "race_result",
    "race_result_text",
    "telemetry_event",
    "telemetry_event_text",
    "capture_end_reason",
    "raw_track_mask_hex",
    "track_mask_hex",
    *[f"ch{index}_raw_white1_black0" for index in range(1, 9)],
    *[f"ch{index}_black" for index in range(1, 9)],
    "active_count",
    "track_ready",
    "armed",
    "manual_speed_mode",
    "marker",
    "recovering",
    "line_fault",
    "marker_cleared",
    "approach_active",
    "marker_clear_streak",
    "marker_confirm_streak",
    "center_confirm_samples",
    "line_lost_samples",
    "line_lost_ms",
    "raw_error",
    "steering_error_used",
    "filtered_error",
    "filtered_derivative",
    "p_term_mps",
    "d_term_mps",
    "steering_correction_mps",
    "requested_base_speed_mps",
    "desired_base_speed_mps",
    "dynamic_base_speed_mps",
    "left_target_mps",
    "right_target_mps",
    "left_speed_mps",
    "right_speed_mps",
    "left_encoder_count",
    "right_encoder_count",
    "left_pwm",
    "right_pwm",
    "mcu_dropped_frames",
]


def sample_to_csv_row(sample: TelemetrySample, end_reason: str = "") -> list[object]:
    raw_channels: Iterable[int] = sample.raw_channels()
    black_channels: Iterable[int] = sample.black_channels()
    return [
        sample.host_time_iso,
        sample.sequence,
        sample.packet_gap,
        sample.board_ms,
        sample.control_tick,
        sample.elapsed_ms,
        sample.state,
        sample.state_text,
        sample.result,
        sample.result_text,
        sample.event,
        sample.event_text,
        end_reason,
        f"0x{sample.raw_track_mask:02X}",
        f"0x{sample.track_mask:02X}",
        *raw_channels,
        *black_channels,
        sample.active_count,
        int(sample.track_ready),
        int(sample.armed),
        int(sample.manual_speed_mode),
        int(sample.marker),
        int(sample.recovering),
        int(sample.line_fault),
        int(sample.marker_cleared),
        int(sample.approach_active),
        sample.marker_clear_streak,
        sample.marker_confirm_streak,
        sample.center_confirm_samples,
        sample.line_lost_samples,
        sample.line_lost_ms,
        f"{sample.raw_error:.2f}",
        f"{sample.steering_error:.2f}",
        f"{sample.filtered_error:.2f}",
        f"{sample.filtered_derivative:.2f}",
        f"{sample.p_term_mps:.4f}",
        f"{sample.d_term_mps:.4f}",
        f"{sample.correction_mps:.4f}",
        f"{sample.requested_base_mps:.4f}",
        f"{sample.desired_base_mps:.4f}",
        f"{sample.dynamic_base_mps:.4f}",
        f"{sample.left_target_mps:.4f}",
        f"{sample.right_target_mps:.4f}",
        f"{sample.left_speed_mps:.4f}",
        f"{sample.right_speed_mps:.4f}",
        sample.left_encoder_count,
        sample.right_encoder_count,
        sample.left_pwm,
        sample.right_pwm,
        sample.mcu_dropped_frames,
    ]
