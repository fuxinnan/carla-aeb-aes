"""AES 规划用的输入和结果。不写入帧日志，也不替代 AEB 的控制请求。"""

from __future__ import annotations

from dataclasses import dataclass


DIRECTION_LEFT = "left"
DIRECTION_RIGHT = "right"
DIRECTIONS = (DIRECTION_LEFT, DIRECTION_RIGHT)

REASON_ROAD = "road_boundary"
REASON_OVERLAP = "target_overlap"
REASON_TOO_CLOSE = "target_too_close"
REASON_LAT_ACCEL = "lat_accel"
REASON_STEER = "steer_angle"
REASON_STEER_RATE = "steer_rate"
REASON_HIGHER_COST = "higher_cost"
REASON_TIE = "equal_cost_keep_left"

REASON_TEXT = {
    REASON_ROAD: "车身超出可行驶边界",
    REASON_OVERLAP: "与目标未来包围盒重叠",
    REASON_TOO_CLOSE: "目标已经太近，横向避让来不及",
    REASON_LAT_ACCEL: "横向加速度超过限值",
    REASON_STEER: "前轮转角超过限值",
    REASON_STEER_RATE: "前轮转角变化率超过限值",
    REASON_HIGHER_COST: "另一侧代价更低",
    REASON_TIE: "两侧代价相同，保留左侧",
}


def direction_text(direction: str) -> str:
    if direction == DIRECTION_LEFT:
        return "向左"
    if direction == DIRECTION_RIGHT:
        return "向右"
    raise ValueError(f"未知方向: {direction}")


@dataclass(frozen=True)
class AesEgo:
    """规划起点。参考点是车辆包围盒中心。"""

    x_m: float  # 单位 m。世界系 X。
    y_m: float  # 单位 m。世界系 Y，向右为正。
    yaw_rad: float  # 单位 rad。世界系航向，俯视顺时针为正。
    speed_mps: float  # 单位 m/s。水平速率，非负。
    length_m: float  # 单位 m。包围盒长度。
    width_m: float  # 单位 m。包围盒宽度。


@dataclass(frozen=True)
class AesTarget:
    """一个目标的当前状态。未来位置按第 4 轮的匀速、航向不变来推。"""

    track_id: str
    x_m: float  # 单位 m。世界系包围盒中心 X。
    y_m: float  # 单位 m。世界系包围盒中心 Y。
    yaw_rad: float  # 单位 rad。世界系航向。
    v_long_mps: float  # 单位 m/s。目标车体系纵向速度，车头为正。
    v_lat_mps: float  # 单位 m/s。目标车体系横向速度，向右为正。
    length_m: float  # 单位 m。
    width_m: float  # 单位 m。


@dataclass(frozen=True)
class DrivableCorridor:
    """可行驶区域。横向相对车道中心，向右为正。source 写明边界从哪来。"""

    left_m: float  # 单位 m。左边界，通常为负。
    right_m: float  # 单位 m。右边界，通常为正。
    source: str

    def __post_init__(self) -> None:
        if self.left_m >= self.right_m:
            raise ValueError("可行驶区域的左边界必须小于右边界")
        if not self.source.strip():
            raise ValueError("可行驶区域必须写明数据来源")


@dataclass(frozen=True)
class TrajectoryPoint:
    """候选轨迹上的一个点。代价是从起点累计到这里的横向加速度平方积分。"""

    t_s: float  # 单位 s。
    x_m: float  # 单位 m。世界系 X。
    y_m: float  # 单位 m。世界系 Y。
    yaw_rad: float  # 单位 rad。路径航向，俯视顺时针为正。
    speed_mps: float  # 单位 m/s。
    a_lat_mps2: float  # 单位 m/s^2。车体横向加速度，向右为正。
    steer_rad: float  # 单位 rad。前轮转角，向右为正。
    steer_rate_radps: float  # 单位 rad/s。相对上一点的前轮转角变化率。
    s_m: float  # 单位 m。沿车道中心线相对起点的弧长。
    cost: float  # 无单位。累计 ∫ a_lat² dt，单位实为 m²/s³。


@dataclass(frozen=True)
class Candidate:
    """一条已经检查过的候选。可行时 reasons 为空。"""

    direction: str
    feasible: bool
    reasons: tuple[str, ...]
    text: str
    cost: float
    points: tuple[TrajectoryPoint, ...]
    peak_lat_accel_mps2: float  # 单位 m/s^2。绝对值峰值。
    peak_steer_rad: float  # 单位 rad。绝对值峰值。
    peak_steer_rate_radps: float  # 单位 rad/s。绝对值峰值。
    min_target_gap_m: float  # 单位 m。与所有目标未来包围盒的最小表面间距。没有目标时为 inf。
    first_overlap_time_s: float | None  # 单位 s。第一次重叠的时间。没有重叠时为 None。


@dataclass(frozen=True)
class Rejection:
    """没有被选用的那一条，以及可读的原因。"""

    direction: str
    reasons: tuple[str, ...]
    text: str


@dataclass(frozen=True)
class AesPlan:
    """一次避让判断。没有安全轨迹时 direction 是 None，不能硬选一条。"""

    avoidable: bool
    direction: str | None
    points: tuple[TrajectoryPoint, ...]
    cost: float | None
    rejections: tuple[Rejection, ...]
    candidates: tuple[Candidate, ...]
    boundary_source: str


@dataclass(frozen=True)
class AesCase:
    """一个离线构造用例。期望结果放在测试里，不放进规划器。"""

    case_id: str
    title: str
    ego: AesEgo
    targets: tuple[AesTarget, ...]
    road: tuple
    corridor: DrivableCorridor
