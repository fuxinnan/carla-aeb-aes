"""目标航迹。真值跟踪只做坐标对齐，不预测、不滤波。"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from scenarios.geometry import box_corners, forward_xy, right_xy
from schemas import EgoState, Track


_TIME_TOLERANCE_S = 1e-4


@dataclass(frozen=True)
class TargetTruth:
    """一帧里某个目标的真值。参考点是它自己的包围盒中心。"""

    track_id: str
    class_name: str
    x_m: float  # 单位 m。世界系 X。
    y_m: float  # 单位 m。世界系 Y。
    z_m: float  # 单位 m。世界系 Z。
    yaw_rad: float  # 单位 rad。世界系航向，0 沿 +X，俯视顺时针为正。
    v_long_mps: float  # 单位 m/s。目标车体系纵向速度，车头为正。
    v_lat_mps: float  # 单位 m/s。目标车体系横向速度，向右为正。
    speed_mps: float  # 单位 m/s。水平速率，非负。
    length_m: float  # 单位 m。沿车头。
    width_m: float  # 单位 m。沿车体横向。
    height_m: float  # 单位 m。沿世界系 Z。
    timestamp_s: float  # 单位 s。这条真值所属的仿真时间。


class BaseTracker(ABC):
    """把一帧观测变成航迹。子类决定数据从哪来。"""

    @abstractmethod
    def update(
        self,
        timestamp_s: float,
        ego: EgoState,
        observations: list[TargetTruth],
        *,
        ego_length_m: float,
        ego_width_m: float,
    ) -> list[Track]:
        """返回与 timestamp_s 对齐的航迹。时间对不上时子类应拒绝这一帧。"""


class GroundTruthTracker(BaseTracker):
    """直接采用目标真值。不做平滑，也不补看不见的目标。"""

    def update(
        self,
        timestamp_s: float,
        ego: EgoState,
        observations: list[TargetTruth],
        *,
        ego_length_m: float,
        ego_width_m: float,
    ) -> list[Track]:
        if ego_length_m <= 0.0 or ego_width_m <= 0.0:
            raise ValueError("自车长宽必须大于 0")
        tracks = [
            _track_from_truth(timestamp_s, ego, observation, ego_length_m, ego_width_m)
            for observation in observations
        ]
        tracks.sort(key=lambda item: item.track_id)
        return tracks


def _track_from_truth(
    timestamp_s: float,
    ego: EgoState,
    observation: TargetTruth,
    ego_length_m: float,
    ego_width_m: float,
) -> Track:
    if abs(observation.timestamp_s - timestamp_s) > _TIME_TOLERANCE_S:
        raise ValueError(
            f"目标 {observation.track_id} 的真值时间 {observation.timestamp_s:.6f} s "
            f"与当前帧 {timestamp_s:.6f} s 不对齐"
        )
    dx = observation.x_m - ego.x_m
    dy = observation.y_m - ego.y_m
    ego_forward = forward_xy(ego.yaw_rad)
    ego_right = right_xy(ego.yaw_rad)
    rel_x = dx * ego_forward[0] + dy * ego_forward[1]
    rel_y = dx * ego_right[0] + dy * ego_right[1]
    target_velocity = _world_velocity(observation.yaw_rad, observation.v_long_mps, observation.v_lat_mps)
    ego_velocity = _world_velocity(ego.yaw_rad, ego.v_long_mps, ego.v_lat_mps)
    dvx = target_velocity[0] - ego_velocity[0]
    dvy = target_velocity[1] - ego_velocity[1]
    in_path, overlap = path_overlap_at(
        ego.x_m,
        ego.y_m,
        ego.yaw_rad,
        ego_length_m,
        ego_width_m,
        observation,
    )
    return Track(
        track_id=observation.track_id,
        source="gt",
        class_name=observation.class_name,
        x_m=float(observation.x_m),
        y_m=float(observation.y_m),
        z_m=float(observation.z_m),
        yaw_rad=float(observation.yaw_rad),
        v_long_mps=float(observation.v_long_mps),
        v_lat_mps=float(observation.v_lat_mps),
        speed_mps=abs(float(observation.speed_mps)),
        length_m=float(observation.length_m),
        width_m=float(observation.width_m),
        height_m=float(observation.height_m),
        rel_x_m=rel_x,
        rel_y_m=rel_y,
        rel_v_long_mps=dvx * ego_forward[0] + dvy * ego_forward[1],
        rel_v_lat_mps=dvx * ego_right[0] + dvy * ego_right[1],
        in_path=in_path,
        timestamp_s=float(timestamp_s),
    )


def _world_velocity(yaw_rad: float, v_long_mps: float, v_lat_mps: float) -> tuple[float, float]:
    fx, fy = forward_xy(yaw_rad)
    rx, ry = right_xy(yaw_rad)
    return fx * v_long_mps + rx * v_lat_mps, fy * v_long_mps + ry * v_lat_mps


def path_overlap_at(
    ego_x_m: float,
    ego_y_m: float,
    ego_yaw_rad: float,
    ego_length_m: float,
    ego_width_m: float,
    observation: TargetTruth,
) -> tuple[bool, float | None]:
    """目标外形当前是否压进自车车宽走廊，以及重叠宽度占自车车宽的比例。

    走廊沿自车航向贯穿前后。目标整车都落在自车后保险杠后面时不算在路径上。
    """
    corners = box_corners(
        observation.x_m,
        observation.y_m,
        observation.yaw_rad,
        observation.length_m,
        observation.width_m,
    )
    fx, fy = forward_xy(ego_yaw_rad)
    rx, ry = right_xy(ego_yaw_rad)
    longitudinal = []
    lateral = []
    for x_m, y_m in corners:
        dx = x_m - ego_x_m
        dy = y_m - ego_y_m
        longitudinal.append(dx * fx + dy * fy)
        lateral.append(dx * rx + dy * ry)
    half_width = ego_width_m * 0.5
    overlap = min(max(lateral), half_width) - max(min(lateral), -half_width)
    ahead_of_rear = max(longitudinal) > -ego_length_m * 0.5
    if overlap <= 1e-6 or not ahead_of_rear:
        return False, None
    return True, max(0.0, min(1.0, overlap / ego_width_m))
