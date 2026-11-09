"""沿车道中心线生成向左、向右两条五次多项式轨迹。"""

from __future__ import annotations

import math
from dataclasses import replace

from scenarios.geometry import forward_xy, pose_on_road, right_xy, sample_at, wrap_pi

from aes.config import AesConfig
from aes.types import DIRECTION_LEFT, DIRECTION_RIGHT, AesEgo, TrajectoryPoint


def generate_trajectory(
    direction: str,
    ego: AesEgo,
    road,
    config: AesConfig,
) -> tuple[TrajectoryPoint, ...]:
    """从自车当前位置偏移到相邻车道。起点和终点的航向、曲率都回到车道方向。"""
    if direction not in (DIRECTION_LEFT, DIRECTION_RIGHT):
        raise ValueError(f"未知方向: {direction}")
    if ego.speed_mps <= 0.5:
        raise ValueError("车速过低，无法生成避让轨迹")
    spec = config.trajectory
    limits = config.constraints
    s0, d0 = project_xy(road, ego.x_m, ego.y_m)
    road_yaw = sample_at(list(road), s0).yaw_rad
    if abs(wrap_pi(ego.yaw_rad - road_yaw)) > 0.2:
        raise ValueError("自车航向与车道方向差得太多，本轮不生成带初始横摆的轨迹")
    sign = -1.0 if direction == DIRECTION_LEFT else 1.0
    horizon = _horizon_s(ego.speed_mps, spec.longitudinal_accel_mps2, spec.length_m)
    times = _sample_times(horizon, spec.sample_dt_s)
    points: list[TrajectoryPoint] = []
    previous_steer = 0.0
    previous_time = 0.0
    cost = 0.0
    for index, time in enumerate(times):
        distance = _distance_at(ego.speed_mps, spec.longitudinal_accel_mps2, time)
        speed = ego.speed_mps + spec.longitudinal_accel_mps2 * time
        if speed <= 0.5:
            raise ValueError("速度剖面在轨迹结束前停住或倒车")
        tau = min(1.0, distance / spec.length_m)
        shape, slope_tau, curve_tau = _quintic(tau)
        lateral = d0 + sign * spec.lateral_offset_m * shape
        slope = sign * spec.lateral_offset_m * slope_tau / spec.length_m
        curve = sign * spec.lateral_offset_m * curve_tau / spec.length_m**2
        x_m, y_m, _z_m, lane_yaw = pose_on_road(list(road), s0 + distance, lateral)
        yaw = wrap_pi(lane_yaw + math.atan(slope))
        kappa_road = _road_curvature(road, s0 + distance)
        kappa = kappa_road + curve / (1.0 + slope * slope) ** 1.5
        steer = math.atan(limits.wheelbase_m * kappa)
        if index == 0:
            rate = 0.0
        else:
            rate = wrap_pi(steer - previous_steer) / (time - previous_time)
            cost += (speed * speed * kappa) ** 2 * (time - previous_time)
        points.append(
            TrajectoryPoint(
                t_s=time,
                x_m=x_m,
                y_m=y_m,
                yaw_rad=yaw,
                speed_mps=speed,
                a_lat_mps2=speed * speed * kappa,
                steer_rad=steer,
                steer_rate_radps=rate,
                s_m=distance,
                cost=cost,
            )
        )
        previous_steer = steer
        previous_time = time
    return _fill_steer_rates(points)


def _fill_steer_rates(points: list[TrajectoryPoint]) -> tuple[TrajectoryPoint, ...]:
    """首点用向前差分，避免图上在 t=0 出现一个假的零变化率。"""
    if len(points) < 2:
        return tuple(points)
    filled = []
    for index, point in enumerate(points):
        if index == 0:
            nxt = points[1]
            rate = (nxt.steer_rad - point.steer_rad) / (nxt.t_s - point.t_s)
        else:
            prev = points[index - 1]
            rate = (point.steer_rad - prev.steer_rad) / (point.t_s - prev.t_s)
        filled.append(replace(point, steer_rate_radps=rate))
    return tuple(filled)


def project_xy(road, x_m: float, y_m: float) -> tuple[float, float]:
    """把世界坐标投到车道中心线上，返回弧长和向右为正的横向坐标。"""
    samples = list(road)
    if len(samples) < 2:
        raise ValueError("道路采样至少要有两个点")
    best_distance = math.inf
    best = (samples[0].s_m, 0.0)
    for index in range(len(samples) - 1):
        start = samples[index]
        end = samples[index + 1]
        dx = end.x_m - start.x_m
        dy = end.y_m - start.y_m
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            continue
        weight = max(0.0, min(1.0, ((x_m - start.x_m) * dx + (y_m - start.y_m) * dy) / length_sq))
        px = start.x_m + weight * dx
        py = start.y_m + weight * dy
        yaw = wrap_pi(start.yaw_rad + wrap_pi(end.yaw_rad - start.yaw_rad) * weight)
        rx, ry = right_xy(yaw)
        lateral = (x_m - px) * rx + (y_m - py) * ry
        distance = math.hypot(x_m - px, y_m - py)
        if distance < best_distance:
            best_distance = distance
            best = (start.s_m + weight * (end.s_m - start.s_m), lateral)
    return best


def corridor_edges(road, corridor) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """可行驶区域的左右边界折线。左边界用右方向乘左侧横向坐标。"""
    left: list[tuple[float, float]] = []
    right: list[tuple[float, float]] = []
    for sample in road:
        rx, ry = right_xy(sample.yaw_rad)
        left.append((sample.x_m + rx * corridor.left_m, sample.y_m + ry * corridor.left_m))
        right.append((sample.x_m + rx * corridor.right_m, sample.y_m + ry * corridor.right_m))
    return left, right


def world_velocity(target) -> tuple[float, float]:
    """目标车体系速度换成世界系。航向顺时针为正，Y 向右为正。"""
    fx, fy = forward_xy(target.yaw_rad)
    rx, ry = right_xy(target.yaw_rad)
    return (
        fx * target.v_long_mps + rx * target.v_lat_mps,
        fy * target.v_long_mps + ry * target.v_lat_mps,
    )


def _quintic(tau: float) -> tuple[float, float, float]:
    """0 到 1 的五次多项式，以及它对 tau 的一阶、二阶导数。两端斜率和曲率为 0。"""
    bounded = min(1.0, max(0.0, tau))
    t2 = bounded * bounded
    t3 = t2 * bounded
    shape = 10.0 * t3 - 15.0 * t2 * t2 + 6.0 * t2 * t3
    slope = 30.0 * t2 - 60.0 * t3 + 30.0 * t2 * t2
    curve = 60.0 * bounded - 180.0 * t2 + 120.0 * t3
    return shape, slope, curve


def _horizon_s(speed_mps: float, accel_mps2: float, length_m: float) -> float:
    if abs(accel_mps2) < 1e-9:
        return length_m / speed_mps
    discriminant = speed_mps * speed_mps + 2.0 * accel_mps2 * length_m
    if discriminant <= 0.25:
        raise ValueError("速度剖面在轨迹结束前停住或倒车")
    return (-speed_mps + math.sqrt(discriminant)) / accel_mps2


def _distance_at(speed_mps: float, accel_mps2: float, time_s: float) -> float:
    return speed_mps * time_s + 0.5 * accel_mps2 * time_s * time_s


def _sample_times(horizon_s: float, dt_s: float) -> list[float]:
    times = [0.0]
    time = dt_s
    while time < horizon_s - 1e-9:
        times.append(time)
        time += dt_s
    if horizon_s - times[-1] > 1e-9:
        times.append(horizon_s)
    return times


def _road_curvature(road, s_m: float) -> float:
    """车道中心线曲率，向右转弯为正。直线路得到 0。"""
    step = 0.5
    behind = sample_at(list(road), s_m - step)
    ahead = sample_at(list(road), s_m + step)
    return wrap_pi(ahead.yaw_rad - behind.yaw_rad) / (2.0 * step)
