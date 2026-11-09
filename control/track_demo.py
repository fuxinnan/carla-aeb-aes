"""单独让自车跟踪一条圆弧，写出横向误差。不启动 CARLA。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

from scenarios.geometry import forward_xy, wrap_pi

from control.pure_pursuit import PurePursuit
from control.round8 import Round8Config, load_round8_config


@dataclass(frozen=True)
class TrackingSample:
    t_s: float
    x_m: float
    y_m: float
    ref_x_m: float
    ref_y_m: float
    error_m: float


@dataclass(frozen=True)
class TrackingResult:
    samples: tuple[TrackingSample, ...]
    max_abs_error_m: float
    threshold_m: float
    passed: bool


def run_tracking_case(config: Round8Config | None = None, dt_s: float = 0.05) -> TrackingResult:
    """半径和车速来自配置。起步横向偏 0.10 m，看纯追踪能不能拉回来。"""
    loaded = config if config is not None else load_round8_config()
    check = loaded.tracking
    path = right_arc(check.arc_radius_m, check.arc_length_m, step_m=0.5)
    tracker = PurePursuit(loaded.pursuit)
    x_m = 0.0
    y_m = -check.initial_offset_m
    yaw = 0.0
    speed = check.speed_mps
    samples = []
    time = 0.0
    horizon = check.arc_length_m / speed + 1.0
    while time <= horizon + 1e-9:
        command = tracker.command(x_m, y_m, yaw, speed, path, dt_s)
        ref_x, ref_y = _nearest(x_m, y_m, path)
        samples.append(TrackingSample(time, x_m, y_m, ref_x, ref_y, command.path_error_m))
        if _passed_end(x_m, y_m, path):
            break
        yaw_rate = command.yaw_rate_radps
        x_m += forward_xy(yaw)[0] * speed * dt_s
        y_m += forward_xy(yaw)[1] * speed * dt_s
        yaw = wrap_pi(yaw + yaw_rate * dt_s)
        time += dt_s
    peak = max(abs(sample.error_m) for sample in samples)
    return TrackingResult(
        samples=tuple(samples),
        max_abs_error_m=peak,
        threshold_m=check.max_abs_error_m,
        passed=peak <= check.max_abs_error_m,
    )


def right_arc(radius_m: float, length_m: float, step_m: float = 0.5) -> tuple[tuple[float, float], ...]:
    """向右的圆弧。航向顺时针增加，曲率中心在路径右侧。"""
    count = max(2, int(math.ceil(length_m / step_m)) + 1)
    points = []
    for index in range(count):
        distance = min(length_m, index * step_m)
        yaw = distance / radius_m
        points.append((radius_m * math.sin(yaw), radius_m * (1.0 - math.cos(yaw))))
    return tuple(points)


def _nearest(x_m: float, y_m: float, path: tuple[tuple[float, float], ...]) -> tuple[float, float]:
    best = math.inf
    chosen = path[0]
    for x1, y1 in path:
        distance = math.hypot(x_m - x1, y_m - y1)
        if distance < best:
            best = distance
            chosen = (x1, y1)
    return chosen


def _passed_end(x_m: float, y_m: float, path: tuple[tuple[float, float], ...]) -> bool:
    end_x, end_y = path[-1]
    return math.hypot(x_m - end_x, y_m - end_y) < 1.5 and x_m > end_x - 2.0


def main() -> None:
    from viz.coordination import plot_tracking

    result = run_tracking_case()
    output = Path(__file__).resolve().parents[1] / "figures" / "path_tracking.png"
    plot_tracking(result, output)
    print(
        f"max_abs_error_m={result.max_abs_error_m:.3f} threshold_m={result.threshold_m:.3f} passed={result.passed} figure={output}",
        flush=True,
    )
    if not result.passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
