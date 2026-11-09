"""第 7 轮 AES 规划配置。轨迹参数和动力学限值都从 YAML 读。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


def default_config_path() -> Path:
    return Path(__file__).resolve().parents[1] / "configs" / "aes" / "round7.yaml"


@dataclass(frozen=True)
class TrajectoryConfig:
    lateral_offset_m: float  # 单位 m。向左或向右的目标横向偏移，取正值。向右为正。
    length_m: float  # 单位 m。沿车道中心线展开的纵向长度。
    sample_dt_s: float  # 单位 s。轨迹点的时间间隔。
    longitudinal_accel_mps2: float  # 单位 m/s^2。速度剖面的纵向加速度，向前为正。0 表示保持当前车速。


@dataclass(frozen=True)
class ConstraintConfig:
    max_lat_accel_mps2: float  # 单位 m/s^2。横向加速度绝对值上限，向右为正。
    max_steer_rad: float  # 单位 rad。前轮转角绝对值上限，向右为正。
    max_steer_rate_radps: float  # 单位 rad/s。前轮转角变化率绝对值上限。
    wheelbase_m: float  # 单位 m。自行车模型轴距。
    close_gap_m: float  # 单位 m。当前表面间距短于该值又发生重叠时，记为已经来不及。
    cost_tie_epsilon: float  # 无单位。两侧代价差不大于该值时视为相同。


@dataclass(frozen=True)
class AesConfig:
    version: str
    trajectory: TrajectoryConfig
    constraints: ConstraintConfig
    path: Path | None = None


def load_aes_config(path: str | Path | None = None) -> AesConfig:
    """读 YAML。缺字段、多字段、非正的限值都直接报错。"""
    source = Path(path) if path is not None else default_config_path()
    loaded = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError(f"{source} 的内容必须是映射")
    _require_keys(loaded, {"version", "trajectory", "constraints"}, "AES 配置")
    version = loaded["version"]
    if not isinstance(version, str) or not version.strip():
        raise ValueError("version 必须是非空字符串")
    return AesConfig(
        version=version,
        trajectory=_trajectory(loaded["trajectory"]),
        constraints=_constraints(loaded["constraints"]),
        path=source,
    )


def _trajectory(data) -> TrajectoryConfig:
    _require_keys(
        data,
        {"lateral_offset_m", "length_m", "sample_dt_s", "longitudinal_accel_mps2"},
        "trajectory",
    )
    return TrajectoryConfig(
        lateral_offset_m=_positive(data["lateral_offset_m"], "trajectory.lateral_offset_m"),
        length_m=_positive(data["length_m"], "trajectory.length_m"),
        sample_dt_s=_positive(data["sample_dt_s"], "trajectory.sample_dt_s"),
        longitudinal_accel_mps2=_number(data["longitudinal_accel_mps2"], "trajectory.longitudinal_accel_mps2"),
    )


def _constraints(data) -> ConstraintConfig:
    _require_keys(
        data,
        {
            "max_lat_accel_mps2",
            "max_steer_rad",
            "max_steer_rate_radps",
            "wheelbase_m",
            "close_gap_m",
            "cost_tie_epsilon",
        },
        "constraints",
    )
    epsilon = _number(data["cost_tie_epsilon"], "constraints.cost_tie_epsilon")
    if epsilon < 0.0:
        raise ValueError("constraints.cost_tie_epsilon 不能为负")
    return ConstraintConfig(
        max_lat_accel_mps2=_positive(data["max_lat_accel_mps2"], "constraints.max_lat_accel_mps2"),
        max_steer_rad=_positive(data["max_steer_rad"], "constraints.max_steer_rad"),
        max_steer_rate_radps=_positive(data["max_steer_rate_radps"], "constraints.max_steer_rate_radps"),
        wheelbase_m=_positive(data["wheelbase_m"], "constraints.wheelbase_m"),
        close_gap_m=_positive(data["close_gap_m"], "constraints.close_gap_m"),
        cost_tie_epsilon=epsilon,
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
