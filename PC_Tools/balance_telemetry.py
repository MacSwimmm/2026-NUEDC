#!/usr/bin/env python3
"""F407-Balance USART3 遥测接收、CSV 记录与保守调参诊断。"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable


PROTOCOL_VERSION = 2
SUPPORTED_PROTOCOL_VERSIONS = {1, 2}
TRACK_PROTOCOL_VERSION = 1
SUPPORTED_TRACK_PROTOCOL_VERSIONS = {1}
DEFAULT_BAUD = 115200
HOLD_ORIGIN_STATE = 16
VISION_HOLD_STATE = 10
MOVE_POSITIVE_STATE = 7
MOVE_NEGATIVE_STATE = 8
HOLD_NEGATIVE_STATE = 9
TARGET_LEVELING_STATE = 17
TARGET_WAIT_CAPTURE_STATE = 18
HOLD_CAPTURED_TARGET_STATE = 19
TRAJECTORY_STATES = {
    MOVE_POSITIVE_STATE,
    MOVE_NEGATIVE_STATE,
    HOLD_NEGATIVE_STATE,
}

STATE_NAMES = {
    0: "BOOT_SAFE",
    1: "WAIT_BUTTON_RELEASE",
    2: "IDLE",
    3: "START_QUERY_MOTOR",
    4: "START_SET_ZERO",
    5: "START_VERIFY_ZERO",
    6: "START_ENABLE",
    7: "MOVE_POSITIVE",
    8: "MOVE_NEGATIVE",
    9: "HOLD_NEGATIVE",
    10: "VISION_HOLD",
    11: "RETURN_ZERO",
    12: "ZERO_HOLD_ABORTED",
    13: "FAULT_LATCHED",
    14: "DISABLING",
    15: "DISABLED",
    16: "HOLD_ORIGIN",
    17: "TARGET_LEVELING",
    18: "TARGET_WAIT_CAPTURE",
    19: "HOLD_CAPTURED_TARGET",
}

TRACK_STATE_NAMES = {
    0: "IDLE",
    1: "TASK1_TRACKING",
    2: "TASK1_FINISHED",
    3: "TASK3_BALANCE",
    4: "TASK3_FINISHED",
    5: "TASK4_BALANCE",
    6: "TASK4_FINISHED",
}

TRACK_CAR_MODE_NAMES = {
    0: "STOP",
    1: "RUN",
    2: "ZERO_SPEED_HOLD",
}

FAULT_NAMES = {
    0: "NONE",
    1: "CAMERA_LOST(兼容字段)",
    2: "CAMERA_OUT_OF_RANGE(兼容字段)",
    3: "MOTOR_OWNER_LOST",
    4: "MOTOR_TX_FAILED",
    5: "MOTOR_ACK_TIMEOUT",
    6: "MOTOR_ACK_REJECTED",
    7: "MOTOR_QUERY_INVALID",
    8: "MOTOR_ZERO_FAILED",
    9: "MOTOR_ZERO_VERIFY_FAILED",
    10: "MOTOR_ENABLE_FAILED",
    11: "HARD_LIMIT_REQUEST",
    12: "RETURN_ZERO_FAILED",
    13: "INTERNAL_STATE_ERROR",
}

START_REJECT_NAMES = {
    0: "NONE",
    1: "BUTTON_NOT_RELEASED",
    2: "CAMERA_OFFLINE",
    3: "TARGET_NOT_FOUND",
    4: "COORDINATE_STALE",
    5: "BALL_NOT_AT_ORIGIN",
    6: "BALL_MOVING_TOO_FAST",
    7: "MOTOR_OWNER_BUSY",
    8: "TUNING_NOT_VALID",
    9: "TARGET_NOT_CONFIRMED",
    10: "NOT_HOLDING_ORIGIN",
    11: "TRAJECTORY_ALREADY_USED",
}

TUNING_BITS = {
    0: "kp",
    1: "ki",
    2: "kd",
    3: "velocity_old_weight",
    4: "integral_window_mm",
    5: "integral_output_limit",
    6: "far_kp",
    7: "far_ki",
    8: "soft_min_pulse",
    9: "soft_max_pulse",
    10: "max_slew_pulse_per_s",
    11: "far_kd",
    12: "far_command_centering_gain_s",
    13: "far_max_slew_pulse_per_s",
    14: "start_speed_tolerance_mm_s",
    15: "far_enter_speed_mm_s",
    16: "far_exit_speed_mm_s",
    17: "positive_speed_tolerance_mm_s",
    18: "far_p_target_limit_pulse",
    19: "far_total_target_limit_pulse",
    20: "negative_speed_tolerance_mm_s",
    21: "far_position_thresholds",
    22: "far_max_step_pulse",
    23: "far_exit_confirm_samples",
    24: "far_target_limit_relation",
    25: "positive_near_rate_pulse_s",
    26: "trajectory_gain_or_rate_scale",
    27: "negative_near_rate_pulse_s",
    28: "trajectory_start_gate",
    29: "trajectory_targets",
    30: "trajectory_endpoint_gate",
    31: "command_centering_gain_s 或 Watch快照读取不一致",
}

TEL_V1_RAW_FIELDS = [
    "protocol_version",
    "sequence",
    "tick_ms",
    "state",
    "fault",
    "start_reject_reason",
    "start_ready",
    "camera_online",
    "target_found",
    "last_valid_age_ms",
    "sample_dt_ms",
    "target_mm",
    "position_mm",
    "velocity_x100",
    "error_x100",
    "p_rate_x100",
    "i_rate_x100",
    "d_rate_x100",
    "centering_rate_x100",
    "raw_rate_x100",
    "limited_rate_x100",
    "command_position_x100",
    "final_requested_pulse",
    "last_transmitted_pulse",
    "last_acked_pulse",
    "motor_owner",
    "motor_state",
    "motor_retry_count",
    "motor_ack_status",
    "motor_last_result",
    "motor_pending_valid",
    "motor_tx_fail_count",
    "motor_ack_timeout_count",
    "motor_ack_reject_count",
    "hard_limit_reject_count",
    "vision_lost_streak",
    "vision_hold_count",
    "tuning_error_mask",
    "camera_frame_sequence",
    "camera_valid_sequence",
    "camera_parse_error_count",
    "camera_physical_reject_count",
    "camera_uart_error_count",
]

TEL_V2_EXTRA_RAW_FIELDS = [
    "run_elapsed_ms",
    "trajectory_ready",
    "trajectory_started",
    "trajectory_completed",
    "trajectory_ready_count",
    "completion_time_ms",
    "score_pass",
    "official_endpoint_error_pass",
    "official_time_pass",
    "run_fault_free",
    "positive_turn_position_mm",
    "positive_turn_abs_error_mm",
    "positive_peak_position_mm",
    "positive_overshoot_mm",
    "negative_min_position_mm",
    "negative_overshoot_mm",
    "negative_stable_window_max_abs_error_mm",
]

TEL_RAW_FIELDS = TEL_V1_RAW_FIELDS + TEL_V2_EXTRA_RAW_FIELDS
TEL_FIELDS_BY_VERSION = {
    1: TEL_V1_RAW_FIELDS,
    2: TEL_RAW_FIELDS,
}

SCALED_TEL_FIELDS = {
    "velocity_x100": "velocity_mm_s",
    "error_x100": "error_mm",
    "p_rate_x100": "p_rate_pulse_s",
    "i_rate_x100": "i_rate_pulse_s",
    "d_rate_x100": "d_rate_pulse_s",
    "centering_rate_x100": "centering_rate_pulse_s",
    "raw_rate_x100": "raw_rate_pulse_s",
    "limited_rate_x100": "limited_rate_pulse_s",
    "command_position_x100": "command_position_pulse",
}

TEL_OUTPUT_FIELDS = [
    SCALED_TEL_FIELDS.get(name, name) for name in TEL_RAW_FIELDS
]

CFG_V1_RAW_FIELDS = [
    "protocol_version",
    "tick_ms",
    "kp_x1000",
    "ki_x1000",
    "kd_x1000",
    "centering_gain_x1000",
    "velocity_old_weight_x1000",
    "integral_window_x100",
    "max_slew_x100",
    "max_step_pulse",
    "soft_min_pulse",
    "soft_max_pulse",
    "camera_fresh_ms",
    "vision_lost_confirm_frames",
    "vision_continue_ms",
    "hard_min_pulse",
    "hard_max_pulse",
]

CFG_V2_EXTRA_RAW_FIELDS = [
    "positive_target_mm",
    "negative_target_mm",
    "trajectory_start_position_tolerance_mm",
    "trajectory_start_speed_x100",
    "trajectory_start_confirm_samples",
    "positive_position_tolerance_mm",
    "positive_speed_x100",
    "positive_confirm_samples",
    "negative_position_tolerance_mm",
    "negative_speed_x100",
    "negative_stable_ms",
    "trajectory_kp_scale_x1000",
    "trajectory_ki_scale_x1000",
    "trajectory_kd_scale_x1000",
    "trajectory_follow_gain_scale_x1000",
    "trajectory_slew_scale_x1000",
    "trajectory_step_scale_x1000",
    "score_limit_ms",
    "official_error_mm",
]

CFG_RAW_FIELDS = CFG_V1_RAW_FIELDS + CFG_V2_EXTRA_RAW_FIELDS
CFG_FIELDS_BY_VERSION = {
    1: CFG_V1_RAW_FIELDS,
    2: CFG_RAW_FIELDS,
}

CFG_SCALES = {
    "kp_x1000": ("cfg_kp", 1000.0),
    "ki_x1000": ("cfg_ki", 1000.0),
    "kd_x1000": ("cfg_kd", 1000.0),
    "centering_gain_x1000": ("cfg_centering_gain_s", 1000.0),
    "velocity_old_weight_x1000": ("cfg_velocity_old_weight", 1000.0),
    "integral_window_x100": ("cfg_integral_window_mm", 100.0),
    "max_slew_x100": ("cfg_max_slew_pulse_s", 100.0),
    "trajectory_start_speed_x100": (
        "cfg_trajectory_start_speed_tolerance_mm_s",
        100.0,
    ),
    "positive_speed_x100": ("cfg_positive_speed_tolerance_mm_s", 100.0),
    "negative_speed_x100": ("cfg_negative_speed_tolerance_mm_s", 100.0),
    "trajectory_kp_scale_x1000": ("cfg_trajectory_kp_scale", 1000.0),
    "trajectory_ki_scale_x1000": ("cfg_trajectory_ki_scale", 1000.0),
    "trajectory_kd_scale_x1000": ("cfg_trajectory_kd_scale", 1000.0),
    "trajectory_follow_gain_scale_x1000": (
        "cfg_trajectory_follow_gain_scale",
        1000.0,
    ),
    "trajectory_slew_scale_x1000": ("cfg_trajectory_slew_scale", 1000.0),
    "trajectory_step_scale_x1000": ("cfg_trajectory_step_scale", 1000.0),
}

CFG_COPY_FIELDS = {
    "max_step_pulse": "cfg_max_step_pulse",
    "soft_min_pulse": "cfg_soft_min_pulse",
    "soft_max_pulse": "cfg_soft_max_pulse",
    "camera_fresh_ms": "cfg_camera_fresh_ms",
    "vision_lost_confirm_frames": "cfg_vision_lost_confirm_frames",
    "vision_continue_ms": "cfg_vision_continue_ms",
    "hard_min_pulse": "cfg_hard_min_pulse",
    "hard_max_pulse": "cfg_hard_max_pulse",
    "positive_target_mm": "cfg_positive_target_mm",
    "negative_target_mm": "cfg_negative_target_mm",
    "trajectory_start_position_tolerance_mm": (
        "cfg_trajectory_start_position_tolerance_mm"
    ),
    "trajectory_start_confirm_samples": "cfg_trajectory_start_confirm_samples",
    "positive_position_tolerance_mm": "cfg_positive_position_tolerance_mm",
    "positive_confirm_samples": "cfg_positive_confirm_samples",
    "negative_position_tolerance_mm": "cfg_negative_position_tolerance_mm",
    "negative_stable_ms": "cfg_negative_stable_ms",
    "score_limit_ms": "cfg_score_limit_ms",
    "official_error_mm": "cfg_official_error_mm",
}

CFG_OUTPUT_FIELDS = [name for name, _scale in CFG_SCALES.values()] + list(
    CFG_COPY_FIELDS.values()
)
CSV_FIELDS = TEL_OUTPUT_FIELDS + CFG_OUTPUT_FIELDS

TRACK_V1_RAW_FIELDS = [
    "protocol_version",
    "sequence",
    "tick_ms",
    "initialized",
    "state",
    "car_mode",
    "track_bits",
    "line_error",
    "left_speed_x100",
    "right_speed_x100",
    "left_speed_set_x100",
    "right_speed_set_x100",
    "left_pwm",
    "right_pwm",
    "left_encoder_fault",
    "right_encoder_fault",
    "finish_pending",
    "finish_armed",
    "elapsed_time_ms",
    "final_time_ms",
    "fast_start_count",
    "slow_start_count",
    "straight_start_count",
    "balance_not_ready_reject_count",
    "recovery_enter_count",
    "control_count",
]

TRACK_FIELDS_BY_VERSION = {
    1: TRACK_V1_RAW_FIELDS,
}

SCALED_TRACK_FIELDS = {
    "left_speed_x100": "left_speed_count_10ms",
    "right_speed_x100": "right_speed_count_10ms",
    "left_speed_set_x100": "left_speed_set_count_10ms",
    "right_speed_set_x100": "right_speed_set_count_10ms",
}

TRACK_OUTPUT_FIELDS = [
    SCALED_TRACK_FIELDS.get(name, name) for name in TRACK_V1_RAW_FIELDS
]
TRACK_CSV_FIELDS = TRACK_OUTPUT_FIELDS


class FrameError(ValueError):
    pass


def _parse_int_fields(parts: list[str], fields: list[str]) -> dict[str, int]:
    if len(parts) != len(fields):
        raise FrameError(f"字段数错误：收到 {len(parts)}，期望 {len(fields)}")
    try:
        return {name: int(value, 10) for name, value in zip(fields, parts)}
    except ValueError as exc:
        raise FrameError(f"存在非整数字段：{exc}") from exc


def _frame_version(parts: list[str]) -> int:
    if not parts:
        raise FrameError("缺少协议版本")
    try:
        version = int(parts[0], 10)
    except ValueError as exc:
        raise FrameError(f"协议版本不是整数：{parts[0]}") from exc
    if version not in SUPPORTED_PROTOCOL_VERSIONS:
        supported = "/".join(str(item) for item in sorted(SUPPORTED_PROTOCOL_VERSIONS))
        raise FrameError(f"协议版本 {version}，脚本只支持 v{supported}")
    return version


def _track_frame_version(parts: list[str]) -> int:
    if not parts:
        raise FrameError("TRK 缺少协议版本")
    try:
        version = int(parts[0], 10)
    except ValueError as exc:
        raise FrameError(f"TRK 协议版本不是整数：{parts[0]}") from exc
    if version not in SUPPORTED_TRACK_PROTOCOL_VERSIONS:
        supported = "/".join(
            str(item) for item in sorted(SUPPORTED_TRACK_PROTOCOL_VERSIONS)
        )
        raise FrameError(f"TRK 协议版本 {version}，脚本只支持 v{supported}")
    return version


def parse_frame(line: str) -> tuple[str, dict[str, Any]]:
    parts = line.strip().split(",")
    if not parts or not parts[0]:
        raise FrameError("空帧")

    kind = parts[0]
    if kind == "HELLO":
        if len(parts) != 5:
            raise FrameError("HELLO 字段数错误")
        version = _frame_version([parts[2]])
        return kind, {
            "product": parts[1],
            "protocol_version": version,
            "baud": int(parts[3]),
            "sample_period_ms": int(parts[4]),
        }

    if kind == "CFG":
        version = _frame_version(parts[1:])
        raw = _parse_int_fields(parts[1:], CFG_FIELDS_BY_VERSION[version])
        data: dict[str, Any] = {
            "protocol_version": raw["protocol_version"],
            "cfg_tick_ms": raw["tick_ms"],
        }
        for raw_name, (out_name, scale) in CFG_SCALES.items():
            if raw_name in raw:
                data[out_name] = raw[raw_name] / scale
        for raw_name, out_name in CFG_COPY_FIELDS.items():
            if raw_name in raw:
                data[out_name] = raw[raw_name]
        return kind, data

    if kind == "TEL":
        version = _frame_version(parts[1:])
        raw = _parse_int_fields(parts[1:], TEL_FIELDS_BY_VERSION[version])
        data = {}
        for name in TEL_FIELDS_BY_VERSION[version]:
            if name in SCALED_TEL_FIELDS:
                data[SCALED_TEL_FIELDS[name]] = raw[name] / 100.0
            else:
                data[name] = raw[name]
        return kind, data

    if kind == "TRK":
        version = _track_frame_version(parts[1:])
        raw = _parse_int_fields(parts[1:], TRACK_FIELDS_BY_VERSION[version])
        data = {}
        for name in TRACK_FIELDS_BY_VERSION[version]:
            if name in SCALED_TRACK_FIELDS:
                data[SCALED_TRACK_FIELDS[name]] = raw[name] / 100.0
            else:
                data[name] = raw[name]
        return kind, data

    raise FrameError(f"未知帧类型：{kind}")


def percentile(values: Iterable[float], percent: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return math.nan
    if len(ordered) == 1:
        return ordered[0]
    index = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def counter_delta(samples: list[dict[str, Any]], field: str) -> int:
    values = [int(sample.get(field, 0)) for sample in samples]
    return max(values) - min(values) if values else 0


def response_times(samples: list[dict[str, Any]]) -> tuple[list[float], int]:
    completed: list[float] = []
    open_episodes = 0
    start_tick: int | None = None

    for sample in samples:
        if sample.get("state") != HOLD_ORIGIN_STATE:
            start_tick = None
            continue
        if not sample.get("camera_online") or not sample.get("target_found"):
            continue
        error = abs(float(sample.get("error_mm", 0.0)))
        tick = int(sample.get("tick_ms", 0))
        if start_tick is None and error >= 25.0:
            start_tick = tick
        elif start_tick is not None and error <= 10.0:
            completed.append(max(0.0, (tick - start_tick) / 1000.0))
            start_tick = None

    if start_tick is not None:
        open_episodes = 1
    return completed, open_episodes


def decode_tuning_mask(mask: int) -> str:
    names = [name for bit, name in TUNING_BITS.items() if mask & (1 << bit)]
    known_mask = sum(1 << bit for bit in TUNING_BITS)
    unknown = mask & ~known_mask
    if unknown:
        names.append(f"未知位0x{unknown:08X}")
    return ", ".join(names) if names else "无"


def assess_trajectory(
    samples: list[dict[str, Any]], cfg: dict[str, Any]
) -> dict[str, Any] | None:
    """读取 v2 固件评分；对历史 v1 状态日志保留近似离线推算。"""

    trajectory_samples = [
        sample
        for sample in samples
        if int(sample.get("trajectory_started", 0)) != 0
        or int(sample.get("state", -1)) in TRAJECTORY_STATES
    ]
    if not trajectory_samples:
        return None

    last = trajectory_samples[-1]
    v2 = int(last.get("protocol_version", 1)) >= 2 and (
        "trajectory_started" in last
    )
    positive_target = int(cfg.get("cfg_positive_target_mm", 50))
    negative_target = int(cfg.get("cfg_negative_target_mm", -50))
    score_limit_ms = int(cfg.get("cfg_score_limit_ms", 5000))
    official_error_mm = int(cfg.get("cfg_official_error_mm", 10))
    stable_ms = int(cfg.get("cfg_negative_stable_ms", 300))

    if v2:
        completed = bool(int(last.get("trajectory_completed", 0)))
        elapsed_ms = int(
            last.get("completion_time_ms", 0)
            if completed
            else last.get("run_elapsed_ms", 0)
        )
        turn_position = int(last.get("positive_turn_position_mm", 0))
        turn_abs_error = int(last.get("positive_turn_abs_error_mm", 0))
        positive_peak = int(last.get("positive_peak_position_mm", 0))
        negative_min = int(last.get("negative_min_position_mm", 0))
        stable_max_error = int(
            last.get("negative_stable_window_max_abs_error_mm", 0)
        )
        fault_free = bool(int(last.get("run_fault_free", 0)))
        time_pass = bool(int(last.get("official_time_pass", 0))) if completed else (
            elapsed_ms <= score_limit_ms
        )
        endpoint_pass = bool(
            int(last.get("official_endpoint_error_pass", 0))
        ) if completed else False
        score_pass = bool(int(last.get("score_pass", 0))) if completed else False
        source = "固件 v2"
    else:
        positive_samples = [
            sample
            for sample in trajectory_samples
            if int(sample.get("state", -1)) == MOVE_POSITIVE_STATE
        ]
        negative_samples = [
            sample
            for sample in trajectory_samples
            if int(sample.get("state", -1))
            in {MOVE_NEGATIVE_STATE, HOLD_NEGATIVE_STATE}
        ]
        completion_sample = next(
            (
                sample
                for sample in trajectory_samples
                if int(sample.get("state", -1)) == HOLD_NEGATIVE_STATE
            ),
            None,
        )
        completed = completion_sample is not None
        start_tick = int(trajectory_samples[0].get("tick_ms", 0))
        end_tick = int(
            (completion_sample or trajectory_samples[-1]).get("tick_ms", start_tick)
        )
        elapsed_ms = max(0, end_tick - start_tick)
        turn_position = int(
            (positive_samples[-1] if positive_samples else trajectory_samples[0]).get(
                "position_mm", 0
            )
        )
        turn_abs_error = abs(positive_target - turn_position)
        positive_peak = max(
            int(sample.get("position_mm", 0)) for sample in trajectory_samples
        )
        negative_min = min(
            (int(sample.get("position_mm", 0)) for sample in negative_samples),
            default=0,
        )
        stable_window = [
            sample
            for sample in negative_samples
            if end_tick - stable_ms <= int(sample.get("tick_ms", 0)) <= end_tick
        ]
        stable_max_error = max(
            (
                abs(negative_target - int(sample.get("position_mm", 0)))
                for sample in stable_window
            ),
            default=0,
        )
        fault_free = all(int(sample.get("fault", 0)) == 0 for sample in trajectory_samples)
        time_pass = completed and elapsed_ms <= score_limit_ms
        endpoint_pass = completed and (
            turn_abs_error <= official_error_mm
            and positive_peak <= positive_target + official_error_mm
            and negative_min >= negative_target - official_error_mm
            and stable_max_error <= official_error_mm
        )
        score_pass = time_pass and endpoint_pass and fault_free
        source = "历史 v1 日志推算"

    return {
        "source": source,
        "completed": completed,
        "elapsed_ms": elapsed_ms,
        "score_limit_ms": score_limit_ms,
        "official_error_mm": official_error_mm,
        "positive_target_mm": positive_target,
        "negative_target_mm": negative_target,
        "negative_stable_ms": stable_ms,
        "turn_position_mm": turn_position,
        "turn_abs_error_mm": turn_abs_error,
        "positive_peak_mm": positive_peak,
        "negative_min_mm": negative_min,
        "stable_max_error_mm": stable_max_error,
        "time_pass": time_pass,
        "endpoint_pass": endpoint_pass,
        "fault_free": fault_free,
        "score_pass": score_pass,
    }


def format_trajectory_assessment(result: dict[str, Any]) -> tuple[list[str], list[str]]:
    completed = bool(result["completed"])
    elapsed_ms = int(result["elapsed_ms"])
    limit_ms = int(result["score_limit_ms"])
    error_limit = int(result["official_error_mm"])
    lines = [
        f"PA0 轨迹评分（{result['source']}）："
        + ("已完成" if completed else "运行中/未完成")
        + f"，计时 {elapsed_ms / 1000.0:.3f} s。",
        f"+端折返点 {int(result['turn_position_mm']):+d} mm，"
        f"折返点误差 {int(result['turn_abs_error_mm'])} mm，"
        f"正向峰值 {int(result['positive_peak_mm']):+d} mm。",
        f"负向最低点 {int(result['negative_min_mm']):+d} mm，"
        f"最终 {int(result['negative_stable_ms'])} ms 窗口最大误差 "
        f"{int(result['stable_max_error_mm'])} mm。",
    ]
    if completed:
        lines.append(
            "自动评分："
            + ("PASS" if bool(result["score_pass"]) else "FAIL")
            + f"（时间≤{limit_ms / 1000.0:g} s，端点/峰值误差≤{error_limit} mm，且无故障）。"
        )

    advice: list[str] = []
    if not completed:
        if elapsed_ms > limit_ms:
            advice.append(
                f"PA0 轨迹尚未完成且已超过 {limit_ms / 1000.0:g} s；先看状态切换、视觉等待和输出限速。"
            )
        else:
            advice.append("PA0 轨迹尚未完成；继续记录到 HOLD_NEGATIVE 稳定判定结束。")
        return lines, advice

    if not bool(result["time_pass"]):
        advice.append(
            f"轨迹用时 {elapsed_ms / 1000.0:.3f} s，超过 {limit_ms / 1000.0:g} s。"
        )
    if int(result["turn_abs_error_mm"]) > error_limit:
        advice.append(
            f"+50 mm 折返点误差为 {int(result['turn_abs_error_mm'])} mm，超过 {error_limit} mm。"
        )
    if int(result["positive_peak_mm"]) > int(result["positive_target_mm"]) + error_limit:
        advice.append(
            f"正向峰值达到 {int(result['positive_peak_mm']):+d} mm，超过允许上界。"
        )
    if int(result["negative_min_mm"]) < int(result["negative_target_mm"]) - error_limit:
        advice.append(
            f"负向最低点达到 {int(result['negative_min_mm']):+d} mm，超过允许下界。"
        )
    if int(result["stable_max_error_mm"]) > error_limit:
        advice.append(
            f"最终稳定窗最大误差 {int(result['stable_max_error_mm'])} mm，超过 {error_limit} mm。"
        )
    if not bool(result["fault_free"]):
        advice.append("轨迹期间出现控制故障，评分无效；先排查故障而不是继续加增益。")
    if bool(result["score_pass"]):
        advice.append("本次 PA0 轨迹自动评分 PASS；保持参数并继续做多次重复性验证。")
    return lines, advice


def analyse(samples: list[dict[str, Any]], cfg: dict[str, Any]) -> tuple[str, list[str]]:
    if len(samples) < 10:
        return "有效 TEL 样本不足，至少记录 1~2 秒。", [
            "先确认 PB10->USB-TTL RX、共地、115200 8N1，并继续采集。"
        ]

    recent = samples[-2400:]  # 20 Hz 下约 120 s
    active = [s for s in recent if s.get("state") == HOLD_ORIGIN_STATE]
    valid = [
        s
        for s in active
        if s.get("camera_online") and s.get("target_found")
    ]
    duration_s = max(
        0.0,
        (int(recent[-1]["tick_ms"]) - int(recent[0]["tick_ms"])) / 1000.0,
    )
    valid_ratio = len(valid) / len(active) if active else 0.0
    errors = [abs(float(s["error_mm"])) for s in valid]
    velocities = [abs(float(s["velocity_mm_s"])) for s in valid]
    times, open_episodes = response_times(recent)
    trajectory_result = assess_trajectory(recent, cfg)
    trajectory_advice: list[str] = []

    summary_lines = [
        f"样本 {len(recent)}，窗口 {duration_s:.1f} s，HOLD_ORIGIN 样本 {len(active)}。",
        f"视觉有效率 {valid_ratio * 100:.1f}%（仅统计 HOLD_ORIGIN）。",
    ]
    if errors:
        summary_lines.append(
            "|误差| 中位/P95/最大 = "
            f"{statistics.median(errors):.1f}/{percentile(errors, 95):.1f}/{max(errors):.1f} mm。"
        )
    if times:
        summary_lines.append(
            "25 mm 外扰回到 ±10 mm："
            f"中位 {statistics.median(times):.2f} s，最慢 {max(times):.2f} s，"
            f"完成 {len(times)} 次。"
        )
    elif open_episodes:
        summary_lines.append("检测到尚未回到 ±10 mm 的外扰过程。")

    if trajectory_result is not None:
        trajectory_lines, trajectory_advice = format_trajectory_assessment(
            trajectory_result
        )
        summary_lines.extend(trajectory_lines)

    advice: list[str] = list(trajectory_advice)
    unsafe = False

    last = recent[-1]
    mask = max(int(s.get("tuning_error_mask", 0)) for s in recent)
    if mask:
        unsafe = True
        advice.append(
            f"先修正无效调参：tuning_error_mask=0x{mask:08X}（{decode_tuning_mask(mask)}）。"
        )

    faults = sorted({int(s.get("fault", 0)) for s in recent if int(s.get("fault", 0))})
    if faults:
        unsafe = True
        names = ", ".join(FAULT_NAMES.get(value, str(value)) for value in faults)
        advice.append(f"出现控制故障 {names}；先清故障根因，不要继续增益调参。")

    motor_faults = {
        "motor_tx_fail_count": "电机发送失败",
        "motor_ack_timeout_count": "电机 ACK 超时",
        "motor_ack_reject_count": "电机拒绝",
    }
    for field, label in motor_faults.items():
        delta = counter_delta(recent, field)
        if delta:
            unsafe = True
            advice.append(f"窗口内{label}增加 {delta} 次；先查 USART1 接线、共地、115200 和 ACK。")

    hard_delta = counter_delta(recent, "hard_limit_reject_count")
    if hard_delta:
        unsafe = True
        advice.append(
            f"硬限位拒绝增加 {hard_delta} 次；核对真实水平零点和回中项，当前边界为 -340/+330 pulse。"
        )

    parse_delta = counter_delta(recent, "camera_parse_error_count")
    physical_delta = counter_delta(recent, "camera_physical_reject_count")
    camera_uart_delta = counter_delta(recent, "camera_uart_error_count")
    if active and (valid_ratio < 0.90 or parse_delta or physical_delta or camera_uart_delta):
        unsafe = True
        advice.append(
            "视觉链路不够稳定："
            f"有效率 {valid_ratio * 100:.1f}%，解析错误 +{parse_delta}，"
            f"物理越界 +{physical_delta}，UART 错误 +{camera_uart_delta}；先修 USART2/视觉。"
        )

    direction_samples = [
        s
        for s in valid
        if abs(float(s["position_mm"])) >= 15.0
        and abs(float(s["velocity_mm_s"])) >= 5.0
    ]
    if len(direction_samples) >= 20:
        away = sum(
            float(s["position_mm"]) * float(s["velocity_mm_s"]) > 0.0
            for s in direction_samples
        )
        away_ratio = away / len(direction_samples)
        if away_ratio >= 0.65:
            unsafe = True
            advice.append(
                f"大误差时有 {away_ratio * 100:.0f}% 样本在远离 O 点；立即停机，先核对摄像头 X 正负和电机正脉冲方向，不能靠负 Kp 修正。"
            )

    if not active:
        if trajectory_result is not None:
            return "\n".join(summary_lines), advice
        state = int(last.get("state", -1))
        reject = int(last.get("start_reject_reason", 0))
        advice.append(
            f"尚未进入 HOLD_ORIGIN：当前 {STATE_NAMES.get(state, state)}，"
            f"start_reject={START_REJECT_NAMES.get(reject, reject)}。"
        )
        return "\n".join(summary_lines), advice

    if valid:
        limited_fraction = sum(
            abs(float(s["raw_rate_pulse_s"]) - float(s["limited_rate_pulse_s"]))
            > max(5.0, abs(float(s["raw_rate_pulse_s"])) * 0.05)
            for s in valid
        ) / len(valid)
        soft_min = float(cfg.get("cfg_soft_min_pulse", -340))
        soft_max = float(cfg.get("cfg_soft_max_pulse", 320))
        at_limit_fraction = sum(
            float(s["command_position_pulse"]) <= soft_min + 3.0
            or float(s["command_position_pulse"]) >= soft_max - 3.0
            for s in valid
        ) / len(valid)

        if limited_fraction >= 0.25:
            median_dt = statistics.median(
                [max(1.0, float(s["sample_dt_ms"])) for s in valid]
            )
            max_step = float(cfg.get("cfg_max_step_pulse", 0))
            max_slew = float(cfg.get("cfg_max_slew_pulse_s", 0))
            step_rate = max_step * 1000.0 / median_dt if max_step else math.inf
            bottleneck = "max_step_pulse" if step_rate < max_slew else "max_slew_pulse_per_s"
            advice.append(
                f"{limited_fraction * 100:.0f}% 有效样本被输出限速，当前主要瓶颈倾向 {bottleneck}；先不要继续加 Kp，低速实车逐级验证限速。"
            )

        if at_limit_fraction >= 0.10:
            advice.append(
                f"命令位置有 {at_limit_fraction * 100:.0f}% 时间贴近软/硬边界；检查水平零点、直接 PD 目标和跟随限速，不能扩大机械行程。"
            )

        signs: list[int] = []
        for sample in valid:
            position = float(sample["position_mm"])
            sign = 1 if position >= 5.0 else -1 if position <= -5.0 else 0
            if sign and (not signs or sign != signs[-1]):
                signs.append(sign)
        crossings = max(0, len(signs) - 1)
        crossing_rate = crossings / max(duration_s, 1.0) * 10.0
        oscillating = crossing_rate >= 3.0 and percentile(errors, 95) > 10.0

        slow = bool(times and statistics.median(times) > 5.0) or (
            open_episodes and duration_s >= 5.0 and abs(float(last["error_mm"])) > 10.0
        )
        if not unsafe and slow and limited_fraction < 0.25:
            kp = float(cfg.get("cfg_kp", 0.0))
            if 0.0 < kp < 8.0:
                candidate = min(8.0, round(kp * 1.10, 2))
                advice.append(
                    f"回原点偏慢且没有明显限速：可只把 Kp 从 {kp:g} 小步试到约 {candidate:g}，其余参数不变，并重新记录同样外扰。"
                )
            else:
                advice.append("回原点偏慢但 Kp 已到边界；检查实际倾角、摩擦和回中项，不要继续扩大行程。")

        if not unsafe and oscillating:
            kd = float(cfg.get("cfg_kd", 0.0))
            kp = float(cfg.get("cfg_kp", 0.0))
            if kd <= 0.001:
                advice.append(
                    f"约每 10 s 出现 {crossing_rate:.1f} 次大幅过零，先保持 Ki=0，可从 Kd=0.05 小步加入速度制动。"
                )
            else:
                advice.append(
                    f"存在往复振荡：可先把 Kd 从 {kd:g} 增加约 10%，若仍振荡再把 Kp={kp:g} 回退约 10%。"
                )

        tail = valid[-min(len(valid), 60):]
        if tail:
            signed_offset = statistics.median(float(s["position_mm"]) for s in tail)
            tail_speed = statistics.median(abs(float(s["velocity_mm_s"])) for s in tail)
            tail_positions = [float(s["position_mm"]) for s in tail]
            tail_position_span = percentile(tail_positions, 95) - percentile(
                tail_positions, 5
            )
            tail_d_p95 = percentile(
                [abs(float(s["d_rate_pulse_s"])) for s in tail], 95
            )
            if (
                not unsafe
                and not oscillating
                and abs(signed_offset) > 5.0
                and tail_speed < 5.0
            ):
                ki = float(cfg.get("cfg_ki", 0.0))
                if ki <= 0.001:
                    advice.append(
                        f"末段存在约 {signed_offset:+.1f} mm 稳定静差且速度较低；当前条件积分应从 Ki=0.30 验证，Ki=0.01 在该量级误差下作用过慢。"
                    )
                else:
                    advice.append(
                        f"仍有约 {signed_offset:+.1f} mm 静差；Ki={ki:g} 已启用，先观察 I 分项是否停止增长或贴近限幅，再决定小步增加 I 限幅，避免只继续加 Ki。"
                    )
            if (
                not unsafe
                and len(tail) >= 20
                and tail_speed < 5.0
                and tail_position_span <= 2.0
                and tail_d_p95 > 100.0
            ):
                advice.append(
                    f"尾段坐标仅在约 {tail_position_span:.1f} mm 范围跳变，但 D 分项 P95 达 {tail_d_p95:.0f} pulse/s；这是量化抖动被 D 放大，应使用近零 D 速度软死区，而不是把真实高速过零统一判为视觉噪声。"
                )

    if not advice:
        advice.append("当前窗口未发现明显安全、视觉、通信或控制异常；保留参数，再做多次左右两侧 20~60 mm 外扰验证。")

    return "\n".join(summary_lines), advice


def render_report(samples: list[dict[str, Any]], cfg: dict[str, Any]) -> str:
    summary, advice = analyse(samples, cfg)
    lines = ["F407-Balance 遥测分析", "", summary, "", "建议："]
    lines.extend(f"{index}. {item}" for index, item in enumerate(advice, 1))
    lines.extend(
        [
            "",
            "说明：建议只基于串口数据，不等同于机械方向、真实水平零点或安全限位的硬件证明。",
        ]
    )
    return "\n".join(lines)


def import_serial():
    try:
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
    except ImportError as exc:
        raise SystemExit("缺少 pyserial，请运行：py -m pip install pyserial") from exc
    return serial, list_ports


def choose_port(requested: str | None) -> str:
    _serial, list_ports = import_serial()
    if requested:
        return requested
    ports = list(list_ports.comports())
    if len(ports) == 1:
        return ports[0].device
    if not ports:
        raise SystemExit("没有发现串口，请检查 USB-TTL 和设备管理器。")
    descriptions = "\n".join(f"  {p.device}: {p.description}" for p in ports)
    raise SystemExit(f"发现多个串口，请用 --port 指定：\n{descriptions}")


def list_serial_ports() -> None:
    _serial, list_ports = import_serial()
    ports = list(list_ports.comports())
    if not ports:
        print("没有发现串口。")
        return
    for port in ports:
        print(f"{port.device:8s} {port.description} [{port.hwid}]")


def default_output_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path.cwd() / f"balance_telemetry_{stamp}.csv"


def receive(args: argparse.Namespace) -> int:
    serial, _list_ports = import_serial()
    port = choose_port(args.port)
    output = Path(args.output).expanduser().resolve() if args.output else default_output_path()
    output.parent.mkdir(parents=True, exist_ok=True)
    summary_path = output.with_name(output.stem + "_summary.txt")

    samples: deque[dict[str, Any]] = deque(maxlen=2400)
    cfg: dict[str, Any] = {}
    bad_frames = 0
    unknown_frames = 0
    track_frames = 0
    rows = 0
    last_live = 0.0
    started = time.monotonic()

    print(f"打开 {port} @ {args.baud} 8N1")
    print(f"CSV: {output}")
    print("Ctrl+C 停止并生成建议。")

    try:
        ser = serial.Serial(port=port, baudrate=args.baud, timeout=0.25)
    except serial.SerialException as exc:
        raise SystemExit(f"无法打开 {port}：{exc}") from exc

    with ser, output.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        try:
            while True:
                if args.duration > 0 and time.monotonic() - started >= args.duration:
                    break
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                try:
                    kind, data = parse_frame(line)
                except FrameError:
                    bad_frames += 1
                    continue

                if kind == "HELLO":
                    print(
                        f"设备 {data['product']}，协议 v{data['protocol_version']}，"
                        f"周期 {data['sample_period_ms']} ms"
                    )
                    continue
                if kind == "CFG":
                    cfg.update(data)
                    print(
                        "参数更新："
                        f"Kp={cfg['cfg_kp']:g} Ki={cfg['cfg_ki']:g} Kd={cfg['cfg_kd']:g} "
                        f"max_slew={cfg['cfg_max_slew_pulse_s']:g} max_step={cfg['cfg_max_step_pulse']}"
                    )
                    continue
                if kind == "TRK":
                    track_frames += 1
                    continue
                if kind != "TEL":
                    unknown_frames += 1
                    continue

                row = dict(data)
                row.update({name: cfg.get(name, "") for name in CFG_OUTPUT_FIELDS})
                writer.writerow(row)
                rows += 1
                samples.append(row)
                if rows % 20 == 0:
                    handle.flush()

                now = time.monotonic()
                if not args.quiet and now - last_live >= 1.0:
                    state = STATE_NAMES.get(int(row["state"]), str(row["state"]))
                    print(
                        f"{state:18s} pos={float(row['position_mm']):+6.1f} mm "
                        f"vel={float(row['velocity_mm_s']):+7.1f} mm/s "
                        f"req/ack={int(row['final_requested_pulse']):+4d}/"
                        f"{int(row['last_acked_pulse']):+4d} "
                        f"age={int(row['last_valid_age_ms']):3d} ms fault={int(row['fault'])}"
                    )
                    last_live = now
        except KeyboardInterrupt:
            print("\n停止采集。")

    report = render_report(list(samples), cfg)
    summary_path.write_text(report + "\n", encoding="utf-8-sig")
    print()
    print(report)
    print()
    print(
        f"已保存 {rows} 行；循迹帧 {track_frames}，坏帧 {bad_frames}，"
        f"未知帧 {unknown_frames}；"
        f"分析报告：{summary_path}"
    )
    return 0


def _number(value: str) -> int | float | str:
    if value == "":
        return value
    try:
        return float(value) if any(ch in value for ch in ".eE") else int(value)
    except ValueError:
        return value


def analyse_csv(path: Path) -> int:
    samples: list[dict[str, Any]] = []
    cfg: dict[str, Any] = {}
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            row = {key: _number(value) for key, value in raw.items() if key is not None}
            samples.append(row)
            for name in CFG_OUTPUT_FIELDS:
                if row.get(name, "") != "":
                    cfg[name] = row[name]
    print(render_report(samples[-2400:], cfg))
    return 0


def make_test_tel(**overrides: int) -> str:
    version = int(overrides.pop("protocol_version", PROTOCOL_VERSION))
    fields = TEL_FIELDS_BY_VERSION[version]
    values = {name: 0 for name in fields}
    values.update(
        protocol_version=version,
        state=HOLD_ORIGIN_STATE,
        camera_online=1,
        target_found=1,
        sample_dt_ms=50,
    )
    values.update(overrides)
    return "TEL," + ",".join(str(values[name]) for name in fields)


def make_test_track(**overrides: int) -> str:
    version = int(overrides.pop("protocol_version", TRACK_PROTOCOL_VERSION))
    fields = TRACK_FIELDS_BY_VERSION[version]
    values = {name: 0 for name in fields}
    values.update(
        protocol_version=version,
        initialized=1,
        state=1,
        car_mode=1,
        track_bits=0x18,
    )
    values.update(overrides)
    return "TRK," + ",".join(str(values[name]) for name in fields)


def make_test_cfg(protocol_version: int = PROTOCOL_VERSION) -> str:
    fields = CFG_FIELDS_BY_VERSION[protocol_version]
    values = {name: 0 for name in fields}
    values.update(
        protocol_version=protocol_version,
        kp_x1000=1800,
        ki_x1000=300,
        kd_x1000=1100,
        positive_target_mm=50,
        negative_target_mm=-50,
        negative_stable_ms=300,
        score_limit_ms=5000,
        official_error_mm=10,
    )
    return "CFG," + ",".join(str(values[name]) for name in fields)


def self_test() -> int:
    kind, parsed = parse_frame(
        make_test_tel(sequence=1, tick_ms=50, position_mm=20, velocity_x100=1200)
    )
    assert kind == "TEL"
    assert parsed["velocity_mm_s"] == 12.0

    kind, track = parse_frame(
        make_test_track(
            sequence=7,
            tick_ms=350,
            line_error=-50,
            left_speed_x100=2850,
            right_speed_x100=3025,
            left_speed_set_x100=3000,
            right_speed_set_x100=3000,
            left_pwm=81,
            right_pwm=76,
        )
    )
    assert kind == "TRK"
    assert track["sequence"] == 7
    assert track["track_bits"] == 0x18
    assert track["left_speed_count_10ms"] == 28.5
    assert track["right_speed_set_count_10ms"] == 30.0

    kind, parsed_v1 = parse_frame(
        make_test_tel(
            protocol_version=1,
            sequence=1,
            tick_ms=50,
            position_mm=20,
            velocity_x100=1200,
        )
    )
    assert kind == "TEL"
    assert parsed_v1["protocol_version"] == 1
    assert "trajectory_started" not in parsed_v1

    kind, cfg_v1 = parse_frame(make_test_cfg(1))
    assert kind == "CFG"
    assert cfg_v1["cfg_kp"] == 1.8
    assert "cfg_positive_target_mm" not in cfg_v1

    kind, cfg_v2 = parse_frame(make_test_cfg(2))
    assert kind == "CFG"
    assert cfg_v2["cfg_positive_target_mm"] == 50
    assert cfg_v2["cfg_score_limit_ms"] == 5000

    samples = []
    for index in range(100):
        _kind, sample = parse_frame(
            make_test_tel(
                sequence=index + 1,
                tick_ms=index * 50,
                position_mm=20 + index // 10,
                velocity_x100=1000,
                error_x100=-(20 + index // 10) * 100,
            )
        )
        samples.append(sample)
    _summary, advice = analyse(samples, {"cfg_kp": 6.0})
    assert any("方向" in item or "远离" in item for item in advice)

    static_samples = []
    for index in range(80):
        _kind, sample = parse_frame(
            make_test_tel(
                sequence=index + 1,
                tick_ms=index * 50,
                position_mm=10,
                error_x100=-1000,
                final_requested_pulse=18,
                last_transmitted_pulse=18,
                last_acked_pulse=18,
            )
        )
        static_samples.append(sample)
    _summary, advice = analyse(
        static_samples,
        {"cfg_kp": 1.8, "cfg_ki": 0.0, "cfg_kd": 1.1},
    )
    assert any("Ki=0.30" in item for item in advice)

    quantized_samples = []
    for index in range(80):
        jump = (index % 5) == 0
        position = 4 if ((index // 5) % 2) else 3
        velocity_x100 = (1300 if position == 4 else -1300) if jump else 0
        d_rate_x100 = (28600 if position == 4 else -28600) if jump else 0
        _kind, sample = parse_frame(
            make_test_tel(
                sequence=index + 1,
                tick_ms=index * 50,
                position_mm=position,
                velocity_x100=velocity_x100,
                error_x100=-position * 100,
                d_rate_x100=d_rate_x100,
            )
        )
        quantized_samples.append(sample)
    _summary, advice = analyse(
        quantized_samples,
        {"cfg_kp": 1.8, "cfg_ki": 0.3, "cfg_kd": 1.1},
    )
    assert any("D 速度软死区" in item for item in advice)

    trajectory_samples = []
    for index in range(40):
        state = MOVE_POSITIVE_STATE if index < 12 else (
            MOVE_NEGATIVE_STATE if index < 34 else HOLD_NEGATIVE_STATE
        )
        _kind, sample = parse_frame(
            make_test_tel(
                sequence=index + 1,
                tick_ms=index * 100,
                state=state,
                target_mm=50 if state == MOVE_POSITIVE_STATE else -50,
                position_mm=min(50, index * 5) if index < 12 else max(-52, 50 - (index - 11) * 5),
                trajectory_started=1,
                trajectory_completed=1 if index == 39 else 0,
                run_elapsed_ms=index * 100,
                completion_time_ms=3900 if index == 39 else 0,
                score_pass=1 if index == 39 else 0,
                official_endpoint_error_pass=1 if index == 39 else 0,
                official_time_pass=1 if index == 39 else 0,
                run_fault_free=1,
                positive_turn_position_mm=50 if index == 39 else 0,
                positive_turn_abs_error_mm=0 if index == 39 else 0,
                positive_peak_position_mm=55 if index == 39 else 0,
                positive_overshoot_mm=5 if index == 39 else 0,
                negative_min_position_mm=-55 if index == 39 else 0,
                negative_overshoot_mm=5 if index == 39 else 0,
                negative_stable_window_max_abs_error_mm=5 if index == 39 else 0,
            )
        )
        trajectory_samples.append(sample)
    trajectory = assess_trajectory(
        trajectory_samples,
        {
            "cfg_positive_target_mm": 50,
            "cfg_negative_target_mm": -50,
            "cfg_score_limit_ms": 5000,
            "cfg_official_error_mm": 10,
        },
    )
    assert trajectory is not None and trajectory["score_pass"] is True
    trajectory_lines, _trajectory_advice = format_trajectory_assessment(trajectory)
    assert any("PASS" in line for line in trajectory_lines)

    legacy_trajectory_samples = []
    for index in range(40):
        if index <= 10:
            state = MOVE_POSITIVE_STATE
            position = index * 5
        elif index < 36:
            state = MOVE_NEGATIVE_STATE
            position = max(-50, 50 - (index - 10) * 5)
        else:
            state = HOLD_NEGATIVE_STATE
            position = -50
        _kind, sample = parse_frame(
            make_test_tel(
                protocol_version=1,
                sequence=index + 1,
                tick_ms=index * 100,
                state=state,
                target_mm=50 if state == MOVE_POSITIVE_STATE else -50,
                position_mm=position,
            )
        )
        legacy_trajectory_samples.append(sample)
    legacy_trajectory = assess_trajectory(legacy_trajectory_samples, {})
    assert legacy_trajectory is not None
    assert legacy_trajectory["source"] == "历史 v1 日志推算"
    assert legacy_trajectory["score_pass"] is True
    print("self-test passed")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="接收 F407-Balance USART3 遥测，保存 CSV 并给出保守调参建议。"
    )
    parser.add_argument("--port", help="串口，例如 COM6；只有一个串口时可省略")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--duration", type=float, default=0.0, help="采集秒数，0=直到 Ctrl+C")
    parser.add_argument("--output", help="CSV 输出路径")
    parser.add_argument("--quiet", action="store_true", help="不显示每秒实时状态")
    parser.add_argument("--list", action="store_true", help="列出串口后退出")
    parser.add_argument("--analyze", type=Path, help="离线分析本脚本生成的 CSV")
    parser.add_argument("--self-test", action="store_true", help="运行解析与建议逻辑自检")
    parser.add_argument("--gui", action="store_true", help="启动桌面实时监控界面")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.gui or len(sys.argv) == 1:
        from balance_telemetry_gui import launch_gui

        return launch_gui()
    if args.self_test:
        return self_test()
    if args.list:
        list_serial_ports()
        return 0
    if args.analyze:
        return analyse_csv(args.analyze.expanduser().resolve())
    return receive(args)


if __name__ == "__main__":
    sys.exit(main())
