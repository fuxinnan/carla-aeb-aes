"""检查一条候选轨迹能不能用：路沿、目标未来占用、横向加速度和前轮转角。"""

from __future__ import annotations

import math

from scenarios.geometry import box_corners, surface_gap

from aes.config import AesConfig
from aes.trajectory import project_xy, world_velocity
from aes.types import (
    REASON_LAT_ACCEL,
    REASON_OVERLAP,
    REASON_ROAD,
    REASON_STEER,
    REASON_STEER_RATE,
    REASON_TEXT,
    REASON_TOO_CLOSE,
    AesEgo,
    AesTarget,
    Candidate,
    DrivableCorridor,
    TrajectoryPoint,
)


_GAP_EPS_M = 1e-6
_BOUNDARY_EPS_M = 1e-3


def inspect_candidate(
    direction: str,
    points: tuple[TrajectoryPoint, ...],
    ego: AesEgo,
    targets: tuple[AesTarget, ...],
    road,
    corridor: DrivableCorridor,
    config: AesConfig,
) -> Candidate:
    """收集这条轨迹的全部拒绝原因。都通过时 reasons 为空。"""
    if not points:
        raise ValueError("候选轨迹没有点")
    limits = config.constraints
    reasons: list[str] = []
    texts: list[str] = []
    if _leaves_corridor(points, ego, road, corridor):
        reasons.append(REASON_ROAD)
        texts.append(REASON_TEXT[REASON_ROAD])
    overlap_text, overlap_time, min_gap = _target_conflict(points, ego, targets, limits.close_gap_m)
    reasons.extend(overlap_text[0])
    texts.extend(overlap_text[1])
    peak_lat = max(abs(point.a_lat_mps2) for point in points)
    peak_steer = max(abs(point.steer_rad) for point in points)
    peak_rate = max(abs(point.steer_rate_radps) for point in points)
    if peak_lat > limits.max_lat_accel_mps2:
        reasons.append(REASON_LAT_ACCEL)
        texts.append(
            f"{REASON_TEXT[REASON_LAT_ACCEL]}：峰值 {peak_lat:.2f} m/s²，限值 {limits.max_lat_accel_mps2:.2f} m/s²"
        )
    if peak_steer > limits.max_steer_rad:
        reasons.append(REASON_STEER)
        texts.append(
            f"{REASON_TEXT[REASON_STEER]}：峰值 {peak_steer:.2f} rad，限值 {limits.max_steer_rad:.2f} rad"
        )
    if peak_rate > limits.max_steer_rate_radps:
        reasons.append(REASON_STEER_RATE)
        texts.append(
            f"{REASON_TEXT[REASON_STEER_RATE]}：峰值 {peak_rate:.2f} rad/s，限值 {limits.max_steer_rate_radps:.2f} rad/s"
        )
    return Candidate(
        direction=direction,
        feasible=not reasons,
        reasons=tuple(reasons),
        text="；".join(texts),
        cost=points[-1].cost,
        points=points,
        peak_lat_accel_mps2=peak_lat,
        peak_steer_rad=peak_steer,
        peak_steer_rate_radps=peak_rate,
        min_target_gap_m=min_gap,
        first_overlap_time_s=overlap_time,
    )


def _leaves_corridor(points, ego: AesEgo, road, corridor: DrivableCorridor) -> bool:
    for point in points:
        for x_m, y_m in box_corners(point.x_m, point.y_m, point.yaw_rad, ego.length_m, ego.width_m):
            _s_m, lateral = project_xy(road, x_m, y_m)
            if lateral < corridor.left_m - _BOUNDARY_EPS_M or lateral > corridor.right_m + _BOUNDARY_EPS_M:
                return True
    return False


def _target_conflict(
    points,
    ego: AesEgo,
    targets: tuple[AesTarget, ...],
    close_gap_m: float,
) -> tuple[tuple[list[str], list[str]], float | None, float]:
    if not targets:
        return ([], []), None, math.inf
    min_gap = math.inf
    first_time: float | None = None
    too_close: list[str] = []
    overlapped: list[str] = []
    for target in targets:
        vx, vy = world_velocity(target)
        initial = surface_gap(
            points[0].x_m,
            points[0].y_m,
            points[0].yaw_rad,
            ego.length_m,
            ego.width_m,
            target.x_m,
            target.y_m,
            target.yaw_rad,
            target.length_m,
            target.width_m,
        )
        hit_time: float | None = None
        for point in points:
            gap = surface_gap(
                point.x_m,
                point.y_m,
                point.yaw_rad,
                ego.length_m,
                ego.width_m,
                target.x_m + vx * point.t_s,
                target.y_m + vy * point.t_s,
                target.yaw_rad,
                target.length_m,
                target.width_m,
            )
            min_gap = min(min_gap, gap)
            if gap <= _GAP_EPS_M and hit_time is None:
                hit_time = point.t_s
        if hit_time is None:
            continue
        if first_time is None or hit_time < first_time:
            first_time = hit_time
        if initial < close_gap_m:
            too_close.append(f"{target.track_id} 当前表面间距 {initial:.2f} m，短于 {close_gap_m:.2f} m")
        else:
            overlapped.append(f"{target.track_id} 在 {hit_time:.2f} s")
    reasons: list[str] = []
    texts: list[str] = []
    if too_close:
        reasons.append(REASON_TOO_CLOSE)
        texts.append(f"{REASON_TEXT[REASON_TOO_CLOSE]}：" + "、".join(too_close))
    if overlapped:
        reasons.append(REASON_OVERLAP)
        texts.append(f"{REASON_TEXT[REASON_OVERLAP]}：" + "、".join(overlapped))
    return (reasons, texts), first_time, min_gap
