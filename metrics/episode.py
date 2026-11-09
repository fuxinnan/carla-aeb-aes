"""从一次运行的三个日志文件计算指标。不读策略代码，也不改日志。"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path

ONSET_DECEL_MPS2 = 0.5
BRAKE_STATES = {"PARTIAL", "FULL"}
_REPEAT = re.compile(r"^rep_(\d+)$")


def evaluate_run(run_dir: str | Path) -> dict:
    """读 frames.jsonl、events.jsonl、summary.json，返回一条可写入表格的记录。"""
    root = Path(run_dir)
    frames = _read_jsonl(root / "frames.jsonl")
    events = _read_jsonl(root / "events.jsonl")
    summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))
    if not frames:
        raise ValueError(f"{root} 没有帧")
    spec = _detail(events, "scenario_spec")
    end = spec.get("end") or {}
    if "hard_negative" not in end:
        raise ValueError(f"{root} 的 scenario_spec 没有 end.hard_negative")
    hard_negative = bool(end["hard_negative"])
    collided = bool(summary["collided"])
    relative = _collision_speed(events) if collided else None
    activations, exits, switches = _state_counts(events)
    requested = [_number((frame.get("control_request") or {}).get("target_decel_mps2")) for frame in frames]
    measured = [_measured_decel(frame) for frame in frames]
    return {
        "run_dir": str(root),
        "scene_id": str(summary["scene_id"]),
        "strategy_name": str(summary["strategy_name"]),
        "repeat_index": _repeat_index(root),
        "seed": spec.get("seed"),
        "hard_negative": hard_negative,
        "collided": collided,
        "collision_relative_speed_mps": relative,
        "min_distance_m": summary.get("min_distance_m"),
        "false_brake": _false_brake(hard_negative, frames),
        "activation_count": activations,
        "exit_count": exits,
        "state_switch_count": switches,
        "risk_to_request_frames": _risk_to_request(frames, requested),
        "request_to_decel_frames": _request_to_decel(requested, measured),
        "peak_decel_mps2": summary.get("peak_decel_mps2"),
        "peak_jerk_mps3": _peak_jerk(frames),
        "trigger_distance_m": (spec.get("trigger") or {}).get("distance_m"),
        "longitudinal_distance_m": _longitudinal_distance(spec),
        "outcome": summary.get("outcome"),
    }


def _false_brake(hard_negative: bool, frames: list[dict]) -> bool | None:
    """误制动只在硬负例上有定义。避开碰撞本身不算误制动。"""
    if not hard_negative:
        return None
    for frame in frames:
        request = frame.get("control_request") or {}
        decel = _number(request.get("target_decel_mps2"))
        brake = _number(request.get("brake"))
        if decel >= 0.1 or brake >= 0.05 or frame.get("aeb_state") in BRAKE_STATES:
            return True
    return False


def _risk_to_request(frames: list[dict], requested: list[float]) -> int | None:
    """第一帧有效 TTC 到第一次请求减速度跨过 0.5 m/s² 的帧数差。"""
    risk = next((index for index, frame in enumerate(frames) if (frame.get("threat") or {}).get("ttc_s") is not None), None)
    request = next((index for index, value in enumerate(requested) if value >= ONSET_DECEL_MPS2), None)
    if risk is None or request is None or request < risk:
        return None
    return request - risk


def _request_to_decel(requested: list[float], measured: list[float]) -> int | None:
    """请求减速度到实测减速度先后跨过 0.5 m/s² 的帧数差。实测来自车速差分。"""
    start = next((index for index, value in enumerate(requested) if value >= ONSET_DECEL_MPS2), None)
    if start is None:
        return None
    for index in range(start + 1, len(measured)):
        if measured[index] >= ONSET_DECEL_MPS2:
            return index - start
    return None


def _peak_jerk(frames: list[dict]) -> float | None:
    """实测纵向加速度的最大绝对变化率，单位 m/s³。"""
    peak = None
    for previous, current in zip(frames, frames[1:]):
        dt = float(current["sim_time_s"]) - float(previous["sim_time_s"])
        if dt <= 0.0:
            continue
        jerk = (float(current["ego"]["a_long_mps2"]) - float(previous["ego"]["a_long_mps2"])) / dt
        magnitude = abs(jerk)
        peak = magnitude if peak is None else max(peak, magnitude)
    if peak is None:
        return None
    if not math.isfinite(peak):
        return None
    return peak


def _state_counts(events: list[dict]) -> tuple[int, int, int]:
    activations = 0
    exits = 0
    switches = 0
    for event in events:
        if event.get("event_type") != "aeb_state":
            continue
        detail = event.get("detail") or {}
        if not detail.get("reason"):
            raise ValueError("aeb_state 事件缺少 reason")
        switches += 1
        destination = detail.get("to")
        if destination in BRAKE_STATES:
            activations += 1
        if destination == "EXIT":
            exits += 1
    return activations, exits, switches


def _collision_speed(events: list[dict]) -> float | None:
    for event in events:
        if event.get("event_type") != "collision":
            continue
        speed = (event.get("detail") or {}).get("relative_speed_mps")
        if speed is None:
            return None
        return float(speed)
    return None


def _longitudinal_distance(spec: dict) -> float | None:
    target = spec.get("target") or {}
    if target.get("gap_m") is not None:
        return float(target["gap_m"])
    ego = spec.get("ego") or {}
    crossing = target.get("crossing_s_m")
    origin = ego.get("s_m")
    if crossing is None or origin is None:
        return None
    return float(crossing) - float(origin)


def _measured_decel(frame: dict) -> float:
    accel = float((frame.get("ego") or {}).get("a_long_mps2") or 0.0)
    return max(0.0, -accel)


def _detail(events: list[dict], event_type: str) -> dict:
    for event in events:
        if event.get("event_type") == event_type:
            detail = event.get("detail") or {}
            if not isinstance(detail, dict):
                raise ValueError(f"{event_type} 的 detail 必须是对象")
            return detail
    raise ValueError(f"日志里没有 {event_type}")


def _repeat_index(root: Path) -> int | None:
    matched = _REPEAT.match(root.name)
    if matched is None:
        return None
    return int(matched.group(1))


def _read_jsonl(path: Path) -> list[dict]:
    if not path.is_file():
        raise FileNotFoundError(path)
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def _number(value) -> float:
    if value is None:
        return 0.0
    return float(value)
