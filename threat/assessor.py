"""威胁评估。跟车用纵向追尾 TTC，横穿和切入用匀速包围盒的重叠时刻。

预测假设写在 ThreatResult.prediction：双方保持当前速度，不转向。
没有有效 TTC 时 ttc_s 是 None，并写 invalid_reason，不把 TTC 填成 0。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from scenarios.geometry import box_corners, forward_xy, right_xy, surface_gap, wrap_pi
from schemas import EgoState, ThreatResult, Track


PREDICTION = "constant_velocity_no_steer"

REASON_TEXT = {
    "no_track": "没有目标",
    "not_closing": "纵向不靠近",
    "target_behind": "目标在车后驶离",
    "no_collision_course": "预测轨迹不相交",
    "passed_conflict": "目标已过冲突点",
    "beyond_horizon": "冲突超出预测时域",
}

_FOLLOW_HEADING_RAD = 0.25
_FOLLOW_LATERAL_MPS = 0.25
_CLOSING_EPS_MPS = 1e-3
_TIME_EPS_S = 1e-6
_PAST_EPS_S = 1e-4


@dataclass(frozen=True)
class ThreatAssessor:
    """挑出当前最危险的一条航迹。最大减速度只用于判断刹得住刹不住。"""

    max_decel_mps2: float = 6.0
    horizon_s: float = 8.0

    def __post_init__(self) -> None:
        if self.max_decel_mps2 <= 0.0:
            raise ValueError("最大减速度必须大于 0")
        if self.horizon_s <= 0.0:
            raise ValueError("预测时域必须大于 0")

    def assess(
        self,
        ego: EgoState,
        tracks: list[Track],
        *,
        ego_length_m: float,
        ego_width_m: float,
    ) -> ThreatResult:
        """主威胁。有效 TTC 里取最小的；都无效时取预测间距最小的。"""
        if ego_length_m <= 0.0 or ego_width_m <= 0.0:
            raise ValueError("自车长宽必须大于 0")
        if not tracks:
            return _empty("no_track")
        ranked = [
            _assess_track(self, ego, track, ego_length_m, ego_width_m)
            for track in tracks
        ]
        valid = [item for item in ranked if item.ttc_s is not None]
        if valid:
            return min(valid, key=lambda item: (item.ttc_s, item.threat_id or ""))
        return min(ranked, key=_invalid_key)


def _empty(reason: str) -> ThreatResult:
    return ThreatResult(
        ttc_s=None,
        pred_min_distance_m=None,
        invalid_reason=reason,
        threat_id=None,
        in_path=None,
        closing_speed_mps=None,
        overlap_ratio=None,
        conflict_x_m=None,
        conflict_y_m=None,
        brake_avoidable=None,
        prediction=PREDICTION,
    )


def _invalid_key(result: ThreatResult) -> tuple:
    distance = result.pred_min_distance_m
    return (distance is None, distance if distance is not None else math.inf, result.threat_id or "")


def _assess_track(
    assessor: ThreatAssessor,
    ego: EgoState,
    track: Track,
    ego_length_m: float,
    ego_width_m: float,
) -> ThreatResult:
    motion = _motion(ego, track)
    interval = _box_overlap_interval(motion, ego_length_m, ego_width_m, track.length_m, track.width_m)
    if _is_following(ego, track):
        longitudinal = _longitudinal(assessor, ego, track, motion, ego_length_m, ego_width_m, interval)
        if longitudinal is not None:
            return longitudinal
    return _from_interval(assessor, ego, track, motion, ego_length_m, ego_width_m, interval)


def _is_following(ego: EgoState, track: Track) -> bool:
    """同向、横向上基本没有相对运动，并且目标中心在自车前方。"""
    heading = abs(wrap_pi(track.yaw_rad - ego.yaw_rad))
    return (
        heading <= _FOLLOW_HEADING_RAD
        and abs(track.rel_v_lat_mps) <= _FOLLOW_LATERAL_MPS
        and track.rel_x_m > 0.0
        and bool(track.in_path)
    )


def _longitudinal(
    assessor: ThreatAssessor,
    ego: EgoState,
    track: Track,
    motion: dict,
    ego_length_m: float,
    ego_width_m: float,
    interval: tuple[float, float] | None,
) -> ThreatResult | None:
    """追尾 TTC = 前向保险杠间距 / 靠近速度。靠近速度不够时不填 0。"""
    gap = _bumper_gap(ego, track, ego_length_m)
    closing = -track.rel_v_long_mps
    if interval is None or interval[1] < -_PAST_EPS_S:
        return None
    if gap <= _TIME_EPS_S:
        return _collision_now(assessor, ego, track, motion, ego_length_m, ego_width_m, closing)
    if closing <= _CLOSING_EPS_MPS:
        distance = _surface(motion, ego_length_m, ego_width_m, track.length_m, track.width_m, 0.0)
        return _invalid_track(assessor, ego, track, "not_closing", distance, distance, closing, ego_length_m, ego_width_m)
    ttc = gap / closing
    if interval[0] > _TIME_EPS_S and abs(ttc - interval[0]) > 0.05:
        # 航向有小偏差时，包围盒给出的首次接触更可靠，两者不一致就不用追尾公式。
        return None
    if ttc > assessor.horizon_s:
        distance = _min_future_gap(assessor, motion, ego_length_m, ego_width_m, track.length_m, track.width_m)
        return _invalid_track(assessor, ego, track, "beyond_horizon", distance, distance, closing, ego_length_m, ego_width_m)
    return _valid(assessor, ego, track, motion, ego_length_m, ego_width_m, ttc, closing)


def _from_interval(
    assessor: ThreatAssessor,
    ego: EgoState,
    track: Track,
    motion: dict,
    ego_length_m: float,
    ego_width_m: float,
    interval: tuple[float, float] | None,
) -> ThreatResult:
    closing = -track.rel_v_long_mps
    if interval is not None and interval[0] <= assessor.horizon_s and interval[1] >= -_PAST_EPS_S:
        if interval[0] <= _TIME_EPS_S:
            return _collision_now(assessor, ego, track, motion, ego_length_m, ego_width_m, closing)
        return _valid(assessor, ego, track, motion, ego_length_m, ego_width_m, interval[0], closing)
    predicted = _min_future_gap(assessor, motion, ego_length_m, ego_width_m, track.length_m, track.width_m)
    current = _surface(motion, ego_length_m, ego_width_m, track.length_m, track.width_m, 0.0)
    reason = _miss_reason(ego, track, motion, ego_length_m, ego_width_m, interval, assessor.horizon_s)
    return _invalid_track(assessor, ego, track, reason, predicted, current, closing, ego_length_m, ego_width_m)


def _miss_reason(
    ego: EgoState,
    track: Track,
    motion: dict,
    ego_length_m: float,
    ego_width_m: float,
    interval: tuple[float, float] | None,
    horizon_s: float,
) -> str:
    if interval is not None and interval[0] > horizon_s:
        return "beyond_horizon"
    if _fully_behind(ego, track, ego_length_m):
        return "target_behind"
    corridor = _corridor_interval(motion, ego_width_m, track)
    if corridor is not None and corridor[1] < -_PAST_EPS_S:
        return "passed_conflict"
    if _is_following(ego, track) and -track.rel_v_long_mps <= _CLOSING_EPS_MPS:
        return "not_closing"
    return "no_collision_course"


def _valid(
    assessor: ThreatAssessor,
    ego: EgoState,
    track: Track,
    motion: dict,
    ego_length_m: float,
    ego_width_m: float,
    ttc_s: float,
    closing: float,
) -> ThreatResult:
    conflict_x, conflict_y = _contact_point(motion, ego_length_m, ego_width_m, track.length_m, track.width_m, ttc_s)
    distance = _brake_distance(ego, ego_length_m, conflict_x, conflict_y)
    return ThreatResult(
        ttc_s=float(ttc_s),
        pred_min_distance_m=0.0,
        invalid_reason=None,
        threat_id=track.track_id,
        in_path=track.in_path,
        closing_speed_mps=float(closing),
        overlap_ratio=_overlap_ratio(ego, track, ego_length_m, ego_width_m),
        conflict_x_m=conflict_x,
        conflict_y_m=conflict_y,
        brake_avoidable=_can_brake(ego.speed_mps, distance, assessor.max_decel_mps2),
        prediction=PREDICTION,
    )


def _collision_now(
    assessor: ThreatAssessor,
    ego: EgoState,
    track: Track,
    motion: dict,
    ego_length_m: float,
    ego_width_m: float,
    closing: float,
) -> ThreatResult:
    conflict_x, conflict_y = _contact_point(motion, ego_length_m, ego_width_m, track.length_m, track.width_m, 0.0)
    return ThreatResult(
        ttc_s=0.0,
        pred_min_distance_m=0.0,
        invalid_reason=None,
        threat_id=track.track_id,
        in_path=track.in_path,
        closing_speed_mps=float(closing),
        overlap_ratio=_overlap_ratio(ego, track, ego_length_m, ego_width_m),
        conflict_x_m=conflict_x,
        conflict_y_m=conflict_y,
        brake_avoidable=_can_brake(ego.speed_mps, 0.0, assessor.max_decel_mps2),
        prediction=PREDICTION,
    )


def _invalid_track(
    assessor: ThreatAssessor,
    ego: EgoState,
    track: Track,
    reason: str,
    predicted_m: float,
    brake_distance_m: float,
    closing: float,
    ego_length_m: float,
    ego_width_m: float,
) -> ThreatResult:
    """没有冲突点时，制动判断用的是当前表面间距，不是未来的最小间距。"""
    return ThreatResult(
        ttc_s=None,
        pred_min_distance_m=float(predicted_m),
        invalid_reason=reason,
        threat_id=track.track_id,
        in_path=track.in_path,
        closing_speed_mps=float(closing),
        overlap_ratio=_overlap_ratio(ego, track, ego_length_m, ego_width_m),
        conflict_x_m=None,
        conflict_y_m=None,
        brake_avoidable=_can_brake(ego.speed_mps, brake_distance_m, assessor.max_decel_mps2),
        prediction=PREDICTION,
    )


def _overlap_ratio(ego: EgoState, track: Track, ego_length_m: float, ego_width_m: float) -> float | None:
    """当前车宽走廊上的重叠宽度，除以自车车宽。不在走廊上时为 None。"""
    corners = box_corners(track.x_m, track.y_m, track.yaw_rad, track.length_m, track.width_m)
    fx, fy = forward_xy(ego.yaw_rad)
    rx, ry = right_xy(ego.yaw_rad)
    longitudinal = []
    lateral = []
    for x_m, y_m in corners:
        dx = x_m - ego.x_m
        dy = y_m - ego.y_m
        longitudinal.append(dx * fx + dy * fy)
        lateral.append(dx * rx + dy * ry)
    half_width = ego_width_m * 0.5
    overlap = min(max(lateral), half_width) - max(min(lateral), -half_width)
    if overlap <= 1e-6 or max(longitudinal) <= -ego_length_m * 0.5:
        return None
    return max(0.0, min(1.0, overlap / ego_width_m))


def _can_brake(speed_mps: float, distance_m: float, max_decel_mps2: float) -> bool:
    """刹停距离 v^2 / (2a) 不超过给定距离时，认为制动来得及。"""
    if distance_m <= 1e-6:
        return speed_mps <= 0.05
    return speed_mps * speed_mps <= 2.0 * max_decel_mps2 * distance_m + 1e-6


def _brake_distance(ego: EgoState, ego_length_m: float, conflict_x_m: float, conflict_y_m: float) -> float:
    """冲突点落在自车前方多少米。用车头中心量，不用两车中心距。"""
    fx, fy = forward_xy(ego.yaw_rad)
    along = (conflict_x_m - ego.x_m) * fx + (conflict_y_m - ego.y_m) * fy
    return along - ego_length_m * 0.5


def _bumper_gap(ego: EgoState, track: Track, ego_length_m: float) -> float:
    """自车车头到目标最近纵向边缘的距离。目标在前方且分开时为正。"""
    nearest = _target_longitudinal_extent(ego, track)[0]
    return nearest - ego_length_m * 0.5


def _target_longitudinal_extent(ego: EgoState, track: Track) -> tuple[float, float]:
    fx, fy = forward_xy(ego.yaw_rad)
    rx, ry = right_xy(track.yaw_rad)
    tfx, tfy = forward_xy(track.yaw_rad)
    center = (track.x_m - ego.x_m) * fx + (track.y_m - ego.y_m) * fy
    support = abs(tfx * fx + tfy * fy) * track.length_m * 0.5 + abs(rx * fx + ry * fy) * track.width_m * 0.5
    return center - support, center + support


def _fully_behind(ego: EgoState, track: Track, ego_length_m: float) -> bool:
    """目标最靠前的一点仍在自车后保险杠之后。"""
    return _target_longitudinal_extent(ego, track)[1] < -ego_length_m * 0.5 + 1e-6


def _motion(ego: EgoState, track: Track) -> dict:
    evx, evy = _body_world(ego.yaw_rad, ego.v_long_mps, ego.v_lat_mps)
    tvx, tvy = _body_world(track.yaw_rad, track.v_long_mps, track.v_lat_mps)
    return {
        "ex": ego.x_m,
        "ey": ego.y_m,
        "eyaw": ego.yaw_rad,
        "evx": evx,
        "evy": evy,
        "tx": track.x_m,
        "ty": track.y_m,
        "tyaw": track.yaw_rad,
        "tvx": tvx,
        "tvy": tvy,
    }


def _body_world(yaw_rad: float, v_long_mps: float, v_lat_mps: float) -> tuple[float, float]:
    fx, fy = forward_xy(yaw_rad)
    rx, ry = right_xy(yaw_rad)
    return fx * v_long_mps + rx * v_lat_mps, fy * v_long_mps + ry * v_lat_mps


def _box_overlap_interval(
    motion: dict,
    ego_length_m: float,
    ego_width_m: float,
    target_length_m: float,
    target_width_m: float,
) -> tuple[float, float] | None:
    """两个航向不变的矩形，中心匀速，返回相交的时间区间。没有相交返回 None。"""
    axes = (
        forward_xy(motion["eyaw"]),
        right_xy(motion["eyaw"]),
        forward_xy(motion["tyaw"]),
        right_xy(motion["tyaw"]),
    )
    lo = -math.inf
    hi = math.inf
    rel_x = motion["ex"] - motion["tx"]
    rel_y = motion["ey"] - motion["ty"]
    rel_vx = motion["evx"] - motion["tvx"]
    rel_vy = motion["evy"] - motion["tvy"]
    for axis_x, axis_y in axes:
        radius = _support(motion["eyaw"], ego_length_m, ego_width_m, axis_x, axis_y)
        radius += _support(motion["tyaw"], target_length_m, target_width_m, axis_x, axis_y)
        span = _axis_times(rel_x, rel_y, rel_vx, rel_vy, axis_x, axis_y, radius)
        if span is None:
            return None
        lo = max(lo, span[0])
        hi = min(hi, span[1])
        if lo > hi + 1e-8:
            return None
    return lo, hi


def _corridor_interval(motion: dict, ego_width_m: float, track: Track) -> tuple[float, float] | None:
    """目标外形压在自车航向走廊上的时间。走廊固定在世界上，不跟着车头平移。"""
    rx, ry = right_xy(motion["eyaw"])
    lateral = (motion["tx"] - motion["ex"]) * rx + (motion["ty"] - motion["ey"]) * ry
    lateral_rate = motion["tvx"] * rx + motion["tvy"] * ry
    radius = ego_width_m * 0.5 + _support(track.yaw_rad, track.length_m, track.width_m, rx, ry)
    return _axis_times(lateral, 0.0, lateral_rate, 0.0, 1.0, 0.0, radius)


def _axis_times(
    rel_x: float,
    rel_y: float,
    rel_vx: float,
    rel_vy: float,
    axis_x: float,
    axis_y: float,
    radius: float,
) -> tuple[float, float] | None:
    """|a + b t| <= radius 的时间区间。相对位置在 t=0 已经投到轴上时，rel_y 和 rel_vy 传 0。"""
    offset = rel_x * axis_x + rel_y * axis_y
    rate = rel_vx * axis_x + rel_vy * axis_y
    if abs(rate) <= 1e-9:
        if abs(offset) <= radius + 1e-7:
            return -math.inf, math.inf
        return None
    enter = (-radius - offset) / rate
    leave = (radius - offset) / rate
    return (min(enter, leave), max(enter, leave))


def _support(yaw_rad: float, length_m: float, width_m: float, axis_x: float, axis_y: float) -> float:
    fx, fy = forward_xy(yaw_rad)
    rx, ry = right_xy(yaw_rad)
    return abs(fx * axis_x + fy * axis_y) * length_m * 0.5 + abs(rx * axis_x + ry * axis_y) * width_m * 0.5


def _contact_point(
    motion: dict,
    ego_length_m: float,
    ego_width_m: float,
    target_length_m: float,
    target_width_m: float,
    time_s: float,
) -> tuple[float, float]:
    """首次接触处的中点。整条边贴上时取接触段中点，避免分离轴来回跳。"""
    ex = motion["ex"] + motion["evx"] * time_s
    ey = motion["ey"] + motion["evy"] * time_s
    tx = motion["tx"] + motion["tvx"] * time_s
    ty = motion["ty"] + motion["tvy"] * time_s
    corners_e = box_corners(ex, ey, motion["eyaw"], ego_length_m, ego_width_m)
    corners_t = box_corners(tx, ty, motion["tyaw"], target_length_m, target_width_m)
    best = math.inf
    points: list[tuple[float, float]] = []
    for origin, other in ((corners_e, corners_t), (corners_t, corners_e)):
        for px, py in origin:
            qx, qy, dist = _closest_boundary(px, py, other)
            mid = ((px + qx) * 0.5, (py + qy) * 0.5)
            if dist < best - 1e-6:
                best = dist
                points = [mid]
            elif dist <= best + 1e-6:
                points.append(mid)
    if not points:
        return (ex + tx) * 0.5, (ey + ty) * 0.5
    return (
        sum(item[0] for item in points) / len(points),
        sum(item[1] for item in points) / len(points),
    )


def _closest_boundary(px: float, py: float, corners: list[tuple[float, float]]) -> tuple[float, float, float]:
    best = math.inf
    point = corners[0]
    for index, (x1, y1) in enumerate(corners):
        x2, y2 = corners[(index + 1) % len(corners)]
        qx, qy, dist = _closest_segment(px, py, x1, y1, x2, y2)
        if dist < best:
            best = dist
            point = (qx, qy)
    return point[0], point[1], best


def _closest_segment(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> tuple[float, float, float]:
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return ax, ay, math.hypot(px - ax, py - ay)
    weight = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    qx = ax + weight * dx
    qy = ay + weight * dy
    return qx, qy, math.hypot(px - qx, py - qy)


def _surface(
    motion: dict,
    ego_length_m: float,
    ego_width_m: float,
    target_length_m: float,
    target_width_m: float,
    time_s: float,
) -> float:
    return surface_gap(
        motion["ex"] + motion["evx"] * time_s,
        motion["ey"] + motion["evy"] * time_s,
        motion["eyaw"],
        ego_length_m,
        ego_width_m,
        motion["tx"] + motion["tvx"] * time_s,
        motion["ty"] + motion["tvy"] * time_s,
        motion["tyaw"],
        target_length_m,
        target_width_m,
    )


def _min_future_gap(
    assessor: ThreatAssessor,
    motion: dict,
    ego_length_m: float,
    ego_width_m: float,
    target_length_m: float,
    target_width_m: float,
) -> float:
    steps = max(2, int(math.ceil(assessor.horizon_s / 0.05)))
    best = math.inf
    for index in range(steps + 1):
        moment = assessor.horizon_s * index / steps
        best = min(
            best,
            _surface(motion, ego_length_m, ego_width_m, target_length_m, target_width_m, moment),
        )
    return float(best)
