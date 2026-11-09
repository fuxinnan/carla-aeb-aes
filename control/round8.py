"""第 8 轮路径跟踪和附着圆配置。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


def default_config_path() -> Path:
    return Path(__file__).resolve().parents[1] / "configs" / "control" / "round8.yaml"


@dataclass(frozen=True)
class PursuitLimits:
    lookahead_gain_s: float  # 单位 s。前视距离 = 增益乘车速。
    lookahead_min_m: float  # 单位 m。
    lookahead_max_m: float  # 单位 m。
    max_lat_accel_mps2: float  # 单位 m/s^2。向右为正。
    max_yaw_rate_radps: float  # 单位 rad/s。顺时针为正。
    max_steer_rad: float  # 单位 rad。前轮转角，向右为正。
    max_steer_rate_radps: float  # 单位 rad/s。
    wheelbase_m: float  # 单位 m。


@dataclass(frozen=True)
class FrictionLimits:
    mu: float  # 无量纲。路面附着系数。
    g_mps2: float  # 单位 m/s^2。

    @property
    def max_accel_mps2(self) -> float:
        return self.mu * self.g_mps2


@dataclass(frozen=True)
class TrackingCheck:
    max_abs_error_m: float  # 单位 m。独立跟踪测试允许的最大横向误差绝对值。
    arc_radius_m: float  # 单位 m。
    arc_length_m: float  # 单位 m。
    speed_mps: float  # 单位 m/s。
    initial_offset_m: float  # 单位 m。起步时放在路径左侧，向右为正的坐标系里是负值。


@dataclass(frozen=True)
class Round8Config:
    version: str
    pursuit: PursuitLimits
    friction: FrictionLimits
    tracking: TrackingCheck
    path: Path | None = None


def load_round8_config(path: str | Path | None = None) -> Round8Config:
    source = Path(path) if path is not None else default_config_path()
    loaded = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError(f"{source} 的内容必须是映射")
    _require_keys(loaded, {"version", "pursuit", "friction", "tracking"}, "第 8 轮配置")
    version = loaded["version"]
    if not isinstance(version, str) or not version.strip():
        raise ValueError("version 必须是非空字符串")
    return Round8Config(
        version=version,
        pursuit=_pursuit(loaded["pursuit"]),
        friction=_friction(loaded["friction"]),
        tracking=_tracking(loaded["tracking"]),
        path=source,
    )


def _pursuit(data) -> PursuitLimits:
    keys = {
        "lookahead_gain_s",
        "lookahead_min_m",
        "lookahead_max_m",
        "max_lat_accel_mps2",
        "max_yaw_rate_radps",
        "max_steer_rad",
        "max_steer_rate_radps",
        "wheelbase_m",
    }
    _require_keys(data, keys, "pursuit")
    minimum = _positive(data["lookahead_min_m"], "pursuit.lookahead_min_m")
    maximum = _positive(data["lookahead_max_m"], "pursuit.lookahead_max_m")
    if minimum > maximum:
        raise ValueError("前视距离下限不能大于上限")
    return PursuitLimits(
        lookahead_gain_s=_positive(data["lookahead_gain_s"], "pursuit.lookahead_gain_s"),
        lookahead_min_m=minimum,
        lookahead_max_m=maximum,
        max_lat_accel_mps2=_positive(data["max_lat_accel_mps2"], "pursuit.max_lat_accel_mps2"),
        max_yaw_rate_radps=_positive(data["max_yaw_rate_radps"], "pursuit.max_yaw_rate_radps"),
        max_steer_rad=_positive(data["max_steer_rad"], "pursuit.max_steer_rad"),
        max_steer_rate_radps=_positive(data["max_steer_rate_radps"], "pursuit.max_steer_rate_radps"),
        wheelbase_m=_positive(data["wheelbase_m"], "pursuit.wheelbase_m"),
    )


def _friction(data) -> FrictionLimits:
    _require_keys(data, {"mu", "g_mps2"}, "friction")
    return FrictionLimits(
        mu=_positive(data["mu"], "friction.mu"),
        g_mps2=_positive(data["g_mps2"], "friction.g_mps2"),
    )


def _tracking(data) -> TrackingCheck:
    keys = {"max_abs_error_m", "arc_radius_m", "arc_length_m", "speed_mps", "initial_offset_m"}
    _require_keys(data, keys, "tracking")
    return TrackingCheck(
        max_abs_error_m=_positive(data["max_abs_error_m"], "tracking.max_abs_error_m"),
        arc_radius_m=_positive(data["arc_radius_m"], "tracking.arc_radius_m"),
        arc_length_m=_positive(data["arc_length_m"], "tracking.arc_length_m"),
        speed_mps=_positive(data["speed_mps"], "tracking.speed_mps"),
        initial_offset_m=_non_negative(data["initial_offset_m"], "tracking.initial_offset_m"),
    )


def _require_keys(data, expected: set[str], label: str) -> None:
    if not isinstance(data, dict):
        raise ValueError(f"{label} 必须是映射")
    extra = set(data) - expected
    missing = expected - set(data)
    if extra:
        raise ValueError(f"{label} 含未知字段: {extra}")
    if missing:
        raise ValueError(f"{label} 缺少字段: {missing}")


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
