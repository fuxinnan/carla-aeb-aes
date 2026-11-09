"""道路坐标、朝向和平面包围盒。不连接仿真器。"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class RoadSample:
    """车道中心线上的一个采样点。s 是从这段路起点量起的弧长。"""

    s_m: float
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    lane_half_width_m: float


def forward_xy(yaw_rad: float) -> tuple[float, float]:
    """CARLA 航向：0 沿 +X，俯视顺时针为正。"""
    return math.cos(yaw_rad), math.sin(yaw_rad)


def right_xy(yaw_rad: float) -> tuple[float, float]:
    """车体右侧，对应 CARLA get_right_vector。"""
    return -math.sin(yaw_rad), math.cos(yaw_rad)


def wrap_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def make_straight_road(
    length_m: float = 120.0,
    step_m: float = 1.0,
    lane_half_width_m: float = 1.75,
    *,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    origin_z: float = 0.0,
    yaw_rad: float = 0.0,
) -> list[RoadSample]:
    """测试和预览用的直线车道。正式运行换成地图上的采样。"""
    if length_m <= 0.0 or step_m <= 0.0:
        raise ValueError("道路长度和采样间隔必须大于 0")
    count = int(math.ceil(length_m / step_m))
    fx, fy = forward_xy(yaw_rad)
    samples = []
    for index in range(count + 1):
        distance = min(length_m, index * step_m)
        samples.append(
            RoadSample(
                s_m=distance,
                x_m=origin_x + fx * distance,
                y_m=origin_y + fy * distance,
                z_m=origin_z,
                yaw_rad=yaw_rad,
                lane_half_width_m=lane_half_width_m,
            )
        )
    return samples


def pose_on_road(samples: list[RoadSample], s_m: float, d_m: float) -> tuple[float, float, float, float]:
    """弧长 s、右侧为正的横向 d，换成世界坐标和该处车道航向。"""
    sample = sample_at(samples, s_m)
    rx, ry = right_xy(sample.yaw_rad)
    return (
        sample.x_m + rx * d_m,
        sample.y_m + ry * d_m,
        sample.z_m,
        sample.yaw_rad,
    )


def sample_at(samples: list[RoadSample], s_m: float) -> RoadSample:
    if not samples:
        raise ValueError("道路采样是空的")
    if s_m <= samples[0].s_m or s_m >= samples[-1].s_m:
        base = samples[0] if s_m <= samples[0].s_m else samples[-1]
        extra = s_m - base.s_m
        fx, fy = forward_xy(base.yaw_rad)
        return RoadSample(
            s_m=s_m,
            x_m=base.x_m + fx * extra,
            y_m=base.y_m + fy * extra,
            z_m=base.z_m,
            yaw_rad=base.yaw_rad,
            lane_half_width_m=base.lane_half_width_m,
        )
    low = 0
    high = len(samples) - 1
    while high - low > 1:
        mid = (low + high) // 2
        if samples[mid].s_m <= s_m:
            low = mid
        else:
            high = mid
    left = samples[low]
    right = samples[high]
    span = right.s_m - left.s_m
    if span <= 1e-9:
        return left
    weight = (s_m - left.s_m) / span
    return RoadSample(
        s_m=s_m,
        x_m=left.x_m + (right.x_m - left.x_m) * weight,
        y_m=left.y_m + (right.y_m - left.y_m) * weight,
        z_m=left.z_m + (right.z_m - left.z_m) * weight,
        yaw_rad=left.yaw_rad + wrap_pi(right.yaw_rad - left.yaw_rad) * weight,
        lane_half_width_m=left.lane_half_width_m + (right.lane_half_width_m - left.lane_half_width_m) * weight,
    )


def lane_boundaries(samples: list[RoadSample]) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """车道左右边界。左边界是中心减去右方向乘半宽。"""
    left: list[tuple[float, float]] = []
    right: list[tuple[float, float]] = []
    for sample in samples:
        rx, ry = right_xy(sample.yaw_rad)
        half = sample.lane_half_width_m
        left.append((sample.x_m - rx * half, sample.y_m - ry * half))
        right.append((sample.x_m + rx * half, sample.y_m + ry * half))
    return left, right


def box_corners(x_m: float, y_m: float, yaw_rad: float, length_m: float, width_m: float) -> list[tuple[float, float]]:
    half_length = length_m * 0.5
    half_width = width_m * 0.5
    fx, fy = forward_xy(yaw_rad)
    rx, ry = right_xy(yaw_rad)
    corners = []
    for along, across in (
        (half_length, half_width),
        (half_length, -half_width),
        (-half_length, -half_width),
        (-half_length, half_width),
    ):
        corners.append((x_m + fx * along + rx * across, y_m + fy * along + ry * across))
    return corners


def surface_gap(
    ax: float,
    ay: float,
    ayaw: float,
    a_length: float,
    a_width: float,
    bx: float,
    by: float,
    byaw: float,
    b_length: float,
    b_width: float,
) -> float:
    """两个平面包围盒的表面间距。重叠时为 0。"""
    corners_a = box_corners(ax, ay, ayaw, a_length, a_width)
    corners_b = box_corners(bx, by, byaw, b_length, b_width)
    if _overlaps(corners_a, corners_b):
        return 0.0
    best = math.inf
    for point in corners_a:
        best = min(best, _point_polygon_edge(point, corners_b))
    for point in corners_b:
        best = min(best, _point_polygon_edge(point, corners_a))
    return float(best)


def _overlaps(corners_a: list[tuple[float, float]], corners_b: list[tuple[float, float]]) -> bool:
    for axis_x, axis_y in _axes(corners_a) + _axes(corners_b):
        a0, a1 = _project(corners_a, axis_x, axis_y)
        b0, b1 = _project(corners_b, axis_x, axis_y)
        if a1 < b0 - 1e-7 or b1 < a0 - 1e-7:
            return False
    return True


def _axes(corners: list[tuple[float, float]]) -> list[tuple[float, float]]:
    axes = []
    for index, (x1, y1) in enumerate(corners):
        x2, y2 = corners[(index + 1) % len(corners)]
        edge_x = x2 - x1
        edge_y = y2 - y1
        length = math.hypot(edge_x, edge_y)
        if length <= 1e-9:
            continue
        axes.append((-edge_y / length, edge_x / length))
    return axes


def _project(corners: list[tuple[float, float]], axis_x: float, axis_y: float) -> tuple[float, float]:
    values = [x * axis_x + y * axis_y for x, y in corners]
    return min(values), max(values)


def _point_polygon_edge(point: tuple[float, float], corners: list[tuple[float, float]]) -> float:
    px, py = point
    best = math.inf
    for index, (x1, y1) in enumerate(corners):
        x2, y2 = corners[(index + 1) % len(corners)]
        best = min(best, _point_segment(px, py, x1, y1, x2, y2))
    return best


def _point_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    weight = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + weight * dx), py - (ay + weight * dy))
