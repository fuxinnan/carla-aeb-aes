"""可见性。默认不遮挡；打开之后，更近的目标挡住视线时，远目标对策略不可见。"""

from __future__ import annotations

import math
from dataclasses import replace

from scenarios.geometry import forward_xy, right_xy
from schemas import EgoState, Track


class VisibilityModel:
    """遮挡按自车中心到目标中心的视线是否穿过更近的包围盒来判断。"""

    def __init__(self, enabled: bool = False) -> None:
        self.enabled = enabled
        self._ratios: dict[str, float] = {}
        self.first_seen_s: dict[str, float] = {}

    def apply(self, tracks: list[Track], ego: EgoState | None = None, timestamp_s: float | None = None) -> list[Track]:
        """去掉不可见航迹。未启用或没有自车位姿时原样返回，供旧测试使用。"""
        if not self.enabled or ego is None:
            self._ratios = {track.track_id: 0.0 for track in tracks}
            return tracks
        visible = []
        for track in tracks:
            ratio = _occlusion_ratio(ego, track, tracks)
            self._ratios[track.track_id] = ratio
            if ratio >= 0.5:
                continue
            seen = self.first_seen_s.get(track.track_id)
            if seen is None and timestamp_s is not None:
                seen = float(timestamp_s)
                self.first_seen_s[track.track_id] = seen
            visible.append(track if track.first_seen_s == seen else replace(track, first_seen_s=seen))
        return visible

    def is_visible(self, track: Track) -> bool:
        """最近一次 apply 里，这条航迹是否交给了策略。"""
        return self._ratios.get(track.track_id, 0.0) < 0.5

    def occlusion_ratio(self, track: Track) -> float:
        """被挡住的比例。0 是完全可见，1 是视线被更近的包围盒挡住。没评估过则为 0。"""
        return self._ratios.get(track.track_id, 0.0)


def _occlusion_ratio(ego: EgoState, track: Track, tracks: list[Track]) -> float:
    target_distance = math.hypot(track.x_m - ego.x_m, track.y_m - ego.y_m)
    for other in tracks:
        if other.track_id == track.track_id:
            continue
        other_distance = math.hypot(other.x_m - ego.x_m, other.y_m - ego.y_m)
        if other_distance >= target_distance - 1e-6:
            continue
        if _segment_hits_box(ego.x_m, ego.y_m, track.x_m, track.y_m, other):
            return 1.0
    return 0.0


def _segment_hits_box(x0: float, y0: float, x1: float, y1: float, box: Track) -> bool:
    fx, fy = forward_xy(box.yaw_rad)
    rx, ry = right_xy(box.yaw_rad)

    def local(x_m: float, y_m: float) -> tuple[float, float]:
        dx = x_m - box.x_m
        dy = y_m - box.y_m
        return dx * fx + dy * fy, dx * rx + dy * ry

    ax, ay = local(x0, y0)
    bx, by = local(x1, y1)
    half_length = box.length_m * 0.5
    half_width = box.width_m * 0.5
    return _segment_hits_aabb(ax, ay, bx, by, -half_length, half_length, -half_width, half_width)


def _segment_hits_aabb(
    ax: float,
    ay: float,
    bx: float,
    by: float,
    xmin: float,
    xmax: float,
    ymin: float,
    ymax: float,
) -> bool:
    t0 = 0.0
    t1 = 1.0
    dx = bx - ax
    dy = by - ay
    for p_value, q_value in ((-dx, ax - xmin), (dx, xmax - ax), (-dy, ay - ymin), (dy, ymax - ay)):
        if abs(p_value) <= 1e-12:
            if q_value < 0.0:
                return False
            continue
        ratio = q_value / p_value
        if p_value < 0.0:
            if ratio > t1:
                return False
            if ratio > t0:
                t0 = ratio
        else:
            if ratio < t0:
                return False
            if ratio < t1:
                t1 = ratio
    return True
