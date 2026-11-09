"""状态机 AEB 加 AES。接口仍是 decide(ego, tracks, threat)。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from scenarios.geometry import box_corners, surface_gap

from aeb.state_machine import StateMachineAeb
from aes.planner import plan_avoidance
from aes.trajectory import project_xy
from aes.types import AesEgo, AesTarget, DrivableCorridor
from arbiter.coordinator import arbitration_reason, with_friction
from control.friction import limit_friction
from control.pure_pursuit import PurePursuit, path_complete
from control.round8 import Round8Config, load_round8_config
from aes.config import AesConfig, load_aes_config
from schemas import AES_ACTIVE, AES_EXIT, AES_IDLE, ControlRequest


@dataclass(frozen=True)
class PolicyReport:
    arbitration_reason: str
    aes_state: str
    aes_direction: str | None
    path_error_m: float | None
    road_departure_m: float
    side_gap_m: float
    lateral_accel_mps2: float
    steer_road_rad: float | None
    yaw_rate_radps: float


class AebAesPolicy:
    """制动来得及时只走状态机。来不及而且避让轨迹可行，才跟踪那条轨迹。"""

    name = "aeb_aes"
    version = "round8-0"

    def __init__(
        self,
        aeb_config,
        aes_config: AesConfig | None = None,
        round8_config: Round8Config | None = None,
    ) -> None:
        self.aeb = StateMachineAeb(aeb_config.state_machine)
        self.aes_config = aes_config if aes_config is not None else load_aes_config()
        self.round8 = round8_config if round8_config is not None else load_round8_config()
        self.tracker = PurePursuit(self.round8.pursuit)
        self.aeb_state = self.aeb.aeb_state
        self.aes_state = AES_IDLE
        self.last_transition = None
        self.report: PolicyReport | None = None
        self.road = None
        self.corridor: DrivableCorridor | None = None
        self.ego_length_m = 4.90
        self.ego_width_m = 2.12
        self.dt_s = 0.05
        self._path: tuple[tuple[float, float], ...] | None = None
        self._direction: str | None = None
        self.wheelbase_m = self.round8.pursuit.wheelbase_m
        self.max_steer_rad = self.round8.pursuit.max_steer_rad
        self.max_yaw_rate_radps = self.round8.pursuit.max_yaw_rate_radps

    def bind_scene(self, road, ego_length_m: float, ego_width_m: float, dt_s: float) -> None:
        samples = tuple(road)
        if len(samples) < 2:
            raise ValueError("道路采样至少要有两个点")
        half = float(samples[0].lane_half_width_m)
        width = 3.0 * half
        source = (
            f"道路采样的车道半宽 {half:.2f} m，左右各扩展一条同宽车道，"
            f"可行驶边界 ±{width:.2f} m，横向以向右为正。"
        )
        self.road = samples
        self.corridor = DrivableCorridor(-width, width, source)
        self.ego_length_m = float(ego_length_m)
        self.ego_width_m = float(ego_width_m)
        self.dt_s = float(dt_s)

    def decide(self, ego, tracks, threat) -> ControlRequest:
        if self.road is None or self.corridor is None:
            raise RuntimeError("AebAesPolicy 还没有绑定道路")
        base = self.aeb.decide(ego, tracks, threat)
        self.aeb_state = self.aeb.aeb_state
        self.last_transition = self.aeb.last_transition
        departure, side_gap = _lateral_risk(ego, tracks, self.road, self.corridor, self.ego_length_m, self.ego_width_m)
        decel = 0.0 if base.target_decel_mps2 is None else max(0.0, float(base.target_decel_mps2))
        speed = float(ego.speed_mps)
        finished = False
        if self._path is not None and (speed <= 0.2 or path_complete(ego.x_m, ego.y_m, self._path)):
            finished = True
            self._path = None
            self._direction = None
            self.tracker.reset()
            self.aes_state = AES_EXIT
        just_selected = False
        if not finished and self._path is None and threat.brake_avoidable is False and speed > 0.5:
            just_selected = self._try_latch(ego, tracks)
        tracking = self._path is not None
        reason = arbitration_reason(threat.brake_avoidable, just_selected, tracking and not just_selected, finished)
        steer_rad = 0.0
        yaw_rate = 0.0
        a_lat = 0.0
        path_error = None
        if tracking and self._path is not None:
            command = self.tracker.command(ego.x_m, ego.y_m, ego.yaw_rad, speed, self._path, self.dt_s)
            steer_rad = command.steer_rad
            yaw_rate = command.yaw_rate_radps
            a_lat = command.a_lat_mps2
            path_error = command.path_error_m
            self.aes_state = AES_ACTIVE
        decel, a_lat, limited = limit_friction(decel, a_lat, self.round8.friction.mu, self.round8.friction.g_mps2)
        if limited and speed > 0.5 and tracking:
            steer_rad, yaw_rate, a_lat = _steer_from_lateral(a_lat, speed, self.wheelbase_m, self.max_steer_rad)
        if tracking:
            self.tracker.remember(steer_rad)
        else:
            self.tracker.reset()
        reason = with_friction(reason, limited)
        steer_norm = 0.0 if self.max_steer_rad <= 0.0 else max(-1.0, min(1.0, steer_rad / self.max_steer_rad))
        self.report = PolicyReport(
            arbitration_reason=reason,
            aes_state=self.aes_state,
            aes_direction=self._direction if self.aes_state == AES_ACTIVE else None,
            path_error_m=path_error,
            road_departure_m=departure,
            side_gap_m=side_gap,
            lateral_accel_mps2=a_lat,
            steer_road_rad=steer_rad if tracking else None,
            yaw_rate_radps=yaw_rate,
        )
        if self.aes_state == AES_EXIT:
            self.aes_state = AES_IDLE
        return ControlRequest(
            throttle=0.0,
            brake=0.0,
            steer=steer_norm,
            desired_speed_mps=base.desired_speed_mps,
            target_decel_mps2=decel,
        )

    def _try_latch(self, ego, tracks) -> bool:
        try:
            plan = plan_avoidance(
                AesEgo(ego.x_m, ego.y_m, ego.yaw_rad, ego.speed_mps, self.ego_length_m, self.ego_width_m),
                tuple(_target(track) for track in tracks),
                self.road,
                self.corridor,
                self.aes_config,
            )
        except ValueError:
            return False
        if not plan.avoidable or not plan.points:
            return False
        self._path = tuple((point.x_m, point.y_m) for point in plan.points)
        self._direction = plan.direction
        self.tracker.reset()
        return True


def _target(track) -> AesTarget:
    return AesTarget(
        track_id=track.track_id,
        x_m=track.x_m,
        y_m=track.y_m,
        yaw_rad=track.yaw_rad,
        v_long_mps=track.v_long_mps,
        v_lat_mps=track.v_lat_mps,
        length_m=track.length_m,
        width_m=track.width_m,
    )


def _steer_from_lateral(a_lat: float, speed: float, wheelbase_m: float, max_steer_rad: float) -> tuple[float, float, float]:
    kappa = a_lat / max(speed * speed, 0.25)
    steer = math.atan(wheelbase_m * kappa)
    steer = max(-max_steer_rad, min(max_steer_rad, steer))
    yaw_rate = speed * math.tan(steer) / wheelbase_m
    return steer, yaw_rate, speed * yaw_rate


def _lateral_risk(ego, tracks, road, corridor: DrivableCorridor, length_m: float, width_m: float) -> tuple[float, float]:
    laterals = []
    departure = 0.0
    for x_m, y_m in box_corners(ego.x_m, ego.y_m, ego.yaw_rad, length_m, width_m):
        _s_m, lateral = project_xy(road, x_m, y_m)
        laterals.append(lateral)
        departure = max(departure, lateral - corridor.right_m, corridor.left_m - lateral)
    side = math.inf
    for track in tracks:
        if abs(float(track.rel_y_m)) <= width_m * 0.5 + 0.2:
            continue
        gap = surface_gap(
            ego.x_m,
            ego.y_m,
            ego.yaw_rad,
            length_m,
            width_m,
            track.x_m,
            track.y_m,
            track.yaw_rad,
            track.length_m,
            track.width_m,
        )
        side = min(side, gap)
    if side is math.inf:
        clearance = min(corridor.right_m - max(laterals), min(laterals) - corridor.left_m)
        side = max(0.0, clearance)
    return max(0.0, departure), float(side)
