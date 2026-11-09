"""第 5 轮 AEB 配置。阈值只从 YAML 读，不写死在策略里。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


def default_config_path() -> Path:
    return Path(__file__).resolve().parents[1] / "configs" / "aeb" / "round5.yaml"


@dataclass(frozen=True)
class FixedTtcConfig:
    ttc_brake_s: float  # 单位 s。TTC 低于该值才请求制动。
    target_decel_mps2: float  # 单位 m/s^2。制动请求，为正。


@dataclass(frozen=True)
class StateMachineConfig:
    confirm_frames: int  # 无单位。连续这么多帧有效风险后才允许进入制动。
    min_hold_frames: int  # 无单位。进入制动后，降级和退出至少再保持这么多帧。
    lost_hold_frames: int  # 无单位。目标短暂丢失时，维持当前制动的帧数。
    clear_frames: int  # 无单位。威胁解除后，再持续这么多帧才退出。
    stop_speed_mps: float  # 单位 m/s。不高于该速度视为已经停稳。
    ttc_warn_on_s: float  # 单位 s。进入 WARN 的 TTC 上界。
    ttc_warn_off_s: float  # 单位 s。离开 WARN 的 TTC 下界，高于进入阈值。
    ttc_partial_on_s: float  # 单位 s。进入部分制动的 TTC 上界。
    ttc_partial_off_s: float  # 单位 s。离开部分制动的 TTC 下界。
    ttc_full_on_s: float  # 单位 s。进入全力制动的 TTC 上界。
    ttc_full_off_s: float  # 单位 s。离开全力制动的 TTC 下界。
    partial_decel_mps2: float  # 单位 m/s^2。部分制动的目标减速度，为正。
    full_decel_mps2: float  # 单位 m/s^2。全力制动的目标减速度，为正。


@dataclass(frozen=True)
class ActuatorConfig:
    brake_per_decel: float  # 单位 s^2/m。减速度到制动踏板的线性系数。
    max_brake: float  # 无量纲。制动请求上限，范围 (0, 1]。
    max_decel_mps2: float  # 单位 m/s^2。执行层允许的最大减速度，为正。
    slew_decel_mps3: float  # 单位 m/s^3。减速度变化率上限，为正。


@dataclass(frozen=True)
class AebConfig:
    version: str
    fixed_ttc: FixedTtcConfig
    state_machine: StateMachineConfig
    actuator: ActuatorConfig
    path: Path


def load_aeb_config(path: str | Path | None = None) -> AebConfig:
    """读 YAML。缺字段、多字段、迟滞写反，都直接报错。"""
    source = Path(path) if path is not None else default_config_path()
    loaded = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError(f"{source} 的内容必须是映射")
    _require_keys(loaded, {"version", "fixed_ttc", "state_machine", "actuator"}, "AEB 配置")
    version = loaded["version"]
    if not isinstance(version, str) or not version.strip():
        raise ValueError("version 必须是非空字符串")
    fixed = _fixed(loaded["fixed_ttc"])
    machine = _machine(loaded["state_machine"])
    actuator = _actuator(loaded["actuator"])
    return AebConfig(version=version, fixed_ttc=fixed, state_machine=machine, actuator=actuator, path=source)


def _fixed(data) -> FixedTtcConfig:
    _require_keys(data, {"ttc_brake_s", "target_decel_mps2"}, "fixed_ttc")
    threshold = _positive(data["ttc_brake_s"], "fixed_ttc.ttc_brake_s")
    decel = _positive(data["target_decel_mps2"], "fixed_ttc.target_decel_mps2")
    return FixedTtcConfig(ttc_brake_s=threshold, target_decel_mps2=decel)


def _machine(data) -> StateMachineConfig:
    keys = {
        "confirm_frames",
        "min_hold_frames",
        "lost_hold_frames",
        "clear_frames",
        "stop_speed_mps",
        "ttc_warn_on_s",
        "ttc_warn_off_s",
        "ttc_partial_on_s",
        "ttc_partial_off_s",
        "ttc_full_on_s",
        "ttc_full_off_s",
        "partial_decel_mps2",
        "full_decel_mps2",
    }
    _require_keys(data, keys, "state_machine")
    warn_on = _positive(data["ttc_warn_on_s"], "ttc_warn_on_s")
    warn_off = _positive(data["ttc_warn_off_s"], "ttc_warn_off_s")
    partial_on = _positive(data["ttc_partial_on_s"], "ttc_partial_on_s")
    partial_off = _positive(data["ttc_partial_off_s"], "ttc_partial_off_s")
    full_on = _positive(data["ttc_full_on_s"], "ttc_full_on_s")
    full_off = _positive(data["ttc_full_off_s"], "ttc_full_off_s")
    order = [warn_off, warn_on, partial_off, partial_on, full_off, full_on]
    if any(left <= right for left, right in zip(order, order[1:])):
        raise ValueError("状态机阈值必须满足 warn_off > warn_on > partial_off > partial_on > full_off > full_on > 0")
    return StateMachineConfig(
        confirm_frames=_count(data["confirm_frames"], "confirm_frames"),
        min_hold_frames=_count(data["min_hold_frames"], "min_hold_frames"),
        lost_hold_frames=_count(data["lost_hold_frames"], "lost_hold_frames"),
        clear_frames=_count(data["clear_frames"], "clear_frames"),
        stop_speed_mps=_non_negative(data["stop_speed_mps"], "stop_speed_mps"),
        ttc_warn_on_s=warn_on,
        ttc_warn_off_s=warn_off,
        ttc_partial_on_s=partial_on,
        ttc_partial_off_s=partial_off,
        ttc_full_on_s=full_on,
        ttc_full_off_s=full_off,
        partial_decel_mps2=_positive(data["partial_decel_mps2"], "partial_decel_mps2"),
        full_decel_mps2=_positive(data["full_decel_mps2"], "full_decel_mps2"),
    )


def _actuator(data) -> ActuatorConfig:
    keys = {"brake_per_decel", "max_brake", "max_decel_mps2", "slew_decel_mps3"}
    _require_keys(data, keys, "actuator")
    max_brake = _number(data["max_brake"], "max_brake")
    if not 0.0 < max_brake <= 1.0:
        raise ValueError("max_brake 必须在 (0, 1] 内")
    return ActuatorConfig(
        brake_per_decel=_positive(data["brake_per_decel"], "brake_per_decel"),
        max_brake=max_brake,
        max_decel_mps2=_positive(data["max_decel_mps2"], "max_decel_mps2"),
        slew_decel_mps3=_positive(data["slew_decel_mps3"], "slew_decel_mps3"),
    )


def _require_keys(data, keys: set[str], label: str) -> None:
    if not isinstance(data, dict):
        raise ValueError(f"{label} 必须是映射")
    extra = sorted(set(data) - keys)
    missing = sorted(keys - set(data))
    if extra:
        raise ValueError(f"{label} 含未知字段: {extra}")
    if missing:
        raise ValueError(f"{label} 缺少字段: {missing}")


def _count(value, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{label} 必须是不小于 1 的整数")
    return value


def _number(value, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} 必须是数值")
    return float(value)


def _positive(value, label: str) -> float:
    number = _number(value, label)
    if number <= 0.0:
        raise ValueError(f"{label} 必须大于 0")
    return number


def _non_negative(value, label: str) -> float:
    number = _number(value, label)
    if number < 0.0:
        raise ValueError(f"{label} 不能为负")
    return number
