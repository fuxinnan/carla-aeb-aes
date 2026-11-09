"""四类离线用例。道路是直线采样，可行驶宽度由车道半宽向两侧各扩一条车道。"""

from __future__ import annotations

import math

from scenarios.geometry import make_straight_road

from aes.types import AesCase, AesEgo, AesTarget, DrivableCorridor


LANE_HALF_WIDTH_M = 1.75
DRIVABLE_HALF_WIDTH_M = LANE_HALF_WIDTH_M * 3.0
BOUNDARY_SOURCE = (
    "道路中心线来自 scenarios.geometry.make_straight_road 的直线采样，车道半宽 1.75 m。"
    "可行驶区域在该车道左右各扩展一条同宽车道，左边界 -5.25 m，右边界 5.25 m，横向以向右为正。"
)

EGO_X_M = 8.0
EGO_SPEED_MPS = 10.0
EGO_LENGTH_M = 4.90
EGO_WIDTH_M = 2.12

PED_X_M = 28.0
PED_ABS_Y_M = 5.2
PED_SPEED_MPS = 1.3
PED_SIZE_M = 0.50

LEAD_X_M = 15.5
LEAD_LENGTH_M = 4.90
LEAD_WIDTH_M = 2.12


def build_cases() -> tuple[AesCase, ...]:
    """左侧可避、右侧可避、两侧都不可避、目标已经太近。"""
    road = tuple(make_straight_road(length_m=100.0, step_m=1.0, lane_half_width_m=LANE_HALF_WIDTH_M))
    corridor = DrivableCorridor(-DRIVABLE_HALF_WIDTH_M, DRIVABLE_HALF_WIDTH_M, BOUNDARY_SOURCE)
    ego = _ego()
    from_right = _pedestrian("ped_right", PED_ABS_Y_M, -math.pi / 2.0)
    from_left = _pedestrian("ped_left", -PED_ABS_Y_M, math.pi / 2.0)
    return (
        AesCase("left_clear", "左侧可避让", ego, (from_right,), road, corridor),
        AesCase("right_clear", "右侧可避让", ego, (from_left,), road, corridor),
        AesCase("both_blocked", "两侧都不可避让", ego, (from_right, from_left), road, corridor),
        AesCase("too_close", "目标已太近来不及", ego, (_lead(),), road, corridor),
    )


def _ego() -> AesEgo:
    return AesEgo(
        x_m=EGO_X_M,
        y_m=0.0,
        yaw_rad=0.0,
        speed_mps=EGO_SPEED_MPS,
        length_m=EGO_LENGTH_M,
        width_m=EGO_WIDTH_M,
    )


def _pedestrian(track_id: str, y_m: float, yaw_rad: float) -> AesTarget:
    """行人沿车头方向走向车道中心。右侧行人航向为 -π/2，速度指向 -Y。"""
    return AesTarget(
        track_id=track_id,
        x_m=PED_X_M,
        y_m=y_m,
        yaw_rad=yaw_rad,
        v_long_mps=PED_SPEED_MPS,
        v_lat_mps=0.0,
        length_m=PED_SIZE_M,
        width_m=PED_SIZE_M,
    )


def _lead() -> AesTarget:
    return AesTarget(
        track_id="lead_01",
        x_m=LEAD_X_M,
        y_m=0.0,
        yaw_rad=0.0,
        v_long_mps=0.0,
        v_lat_mps=0.0,
        length_m=LEAD_LENGTH_M,
        width_m=LEAD_WIDTH_M,
    )
