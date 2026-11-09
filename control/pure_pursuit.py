"""纯追踪。曲率直接受横向加速度和横摆角速度限制。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from scenarios.geometry import forward_xy, right_xy, wrap_pi

from control.round8 import PursuitLimits


@dataclass(frozen=True)
class PursuitCommand:
    steer_rad: float  # 单位 rad。向右为正。
    steer_norm: float  # 无量纲，-1~1。1 对应最大前轮转角。
    yaw_rate_radps: float  # 单位 rad/s。
    a_lat_mps2: float  # 单位 m/s^2。向右为正。
    path_error_m: float  # 单位 m。相对路径，向右为正。


class PurePursuit:
    """前视点变成曲率。选它是因为运动学自行车没有侧偏角，曲率就是横摆角速度除以车速。"""

    def __init__(self, limits: PursuitLimits) -> None:
        self.limits = limits
        self._steer_rad = 0.0

    def reset(self) -> None:
        self._steer_rad = 0.0

    def remember(self, steer_rad: float) -> None:
        """附着圆如果改过转角，下一步的变化率从上一个实际转角接着算。"""
        self._steer_rad = float(steer_rad)

    def command(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        speed_mps: float,
        path: tuple[tuple[float, float], ...],
        dt_s: float,
    ) -> PursuitCommand:
        error = cross_track_error(x_m, y_m, path)
        if speed_mps < 0.5 or len(path) < 2 or dt_s <= 0.0:
            self._steer_rad = 0.0
            return PursuitCommand(0.0, 0.0, 0.0, 0.0, error)
        limits = self.limits
        lookahead = min(limits.lookahead_max_m, max(limits.lookahead_min_m, limits.lookahead_gain_s * speed_mps))
        target_x, target_y = lookahead_point(x_m, y_m, path, lookahead)
        alpha = _bearing(x_m, y_m, yaw_rad, target_x, target_y)
        kappa = 2.0 * math.sin(alpha) / lookahead
        a_lat = _clamp(speed_mps * speed_mps * kappa, limits.max_lat_accel_mps2)
        kappa = a_lat / max(speed_mps * speed_mps, 0.25)
        yaw_rate = _clamp(speed_mps * kappa, limits.max_yaw_rate_radps)
        kappa = yaw_rate / max(speed_mps, 0.5)
        steer = _clamp(math.atan(limits.wheelbase_m * kappa), limits.max_steer_rad)
        step = limits.max_steer_rate_radps * dt_s
        steer = min(self._steer_rad + step, max(self._steer_rad - step, steer))
        self._steer_rad = steer
        kappa = math.tan(steer) / limits.wheelbase_m
        yaw_rate = speed_mps * kappa
        a_lat = speed_mps * yaw_rate
        return PursuitCommand(
            steer_rad=steer,
            steer_norm=steer / limits.max_steer_rad,
            yaw_rate_radps=yaw_rate,
            a_lat_mps2=a_lat,
            path_error_m=error,
        )


def cross_track_error(x_m: float, y_m: float, path: tuple[tuple[float, float], ...]) -> float:
    """点到路径的有符号距离。路径右侧为正。"""
    if len(path) < 2:
        return 0.0
    _distance, px, py, tx, ty = _closest(x_m, y_m, path)
    return (x_m - px) * (-ty) + (y_m - py) * tx


def lookahead_point(
    x_m: float,
    y_m: float,
    path: tuple[tuple[float, float], ...],
    lookahead_m: float,
) -> tuple[float, float]:
    """从最近点沿路径向前走前视距离。不够长就用终点。"""
    if len(path) < 2:
        return path[-1]
    seg, px, py, _tx, _ty = _closest(x_m, y_m, path)
    remain = lookahead_m
    cx, cy = px, py
    for index in range(seg, len(path) - 1):
        nx, ny = path[index + 1]
        step = math.hypot(nx - cx, ny - cy)
        if step <= 1e-9:
            cx, cy = nx, ny
            continue
        if step >= remain:
            weight = remain / step
            return (cx + (nx - cx) * weight, cy + (ny - cy) * weight)
        remain -= step
        cx, cy = nx, ny
    return path[-1]


def path_complete(x_m: float, y_m: float, path: tuple[tuple[float, float], ...], finish_m: float = 1.0) -> bool:
    """已经靠近终点，并且终点落在车后或车旁。"""
    if len(path) < 2:
        return True
    end_x, end_y = path[-1]
    if math.hypot(x_m - end_x, y_m - end_y) > finish_m:
        return False
    prev_x, prev_y = path[-2]
    tx, ty = end_x - prev_x, end_y - prev_y
    length = math.hypot(tx, ty)
    if length <= 1e-9:
        return True
    along = ((x_m - prev_x) * tx + (y_m - prev_y) * ty) / length
    return along >= length - finish_m


def _closest(x_m: float, y_m: float, path: tuple[tuple[float, float], ...]):
    best = math.inf
    chosen = (0, path[0][0], path[0][1], 1.0, 0.0)
    for index in range(len(path) - 1):
        x1, y1 = path[index]
        x2, y2 = path[index + 1]
        dx, dy = x2 - x1, y2 - y1
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            continue
        weight = max(0.0, min(1.0, ((x_m - x1) * dx + (y_m - y1) * dy) / length_sq))
        px = x1 + weight * dx
        py = y1 + weight * dy
        distance = math.hypot(x_m - px, y_m - py)
        if distance < best:
            best = distance
            length = math.sqrt(length_sq)
            chosen = (index, px, py, dx / length, dy / length)
    return chosen


def _bearing(x_m: float, y_m: float, yaw_rad: float, target_x: float, target_y: float) -> float:
    dx = target_x - x_m
    dy = target_y - y_m
    fx, fy = forward_xy(yaw_rad)
    rx, ry = right_xy(yaw_rad)
    longitudinal = dx * fx + dy * fy
    lateral = dx * rx + dy * ry
    return math.atan2(lateral, longitudinal)


def _clamp(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def wrap_heading(angle_rad: float) -> float:
    return wrap_pi(angle_rad)
