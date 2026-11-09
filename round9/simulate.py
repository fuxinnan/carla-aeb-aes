"""第 9 轮运动学闭环。直路、多目标，策略和威胁评估沿用现有模块。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from control.actuator import BrakeActuator
from perception.tracker import GroundTruthTracker, TargetTruth
from perception.visibility import VisibilityModel
from round9.catalog import DT_S, EGO_S_M, TIMEOUT_S, GridScene
from scenarios.closed_loop import make_summary
from scenarios.geometry import forward_xy, make_straight_road, right_xy, surface_gap
from schemas import AES_IDLE, EgoState, EpisodeSummary, FrameRecord
from threat.assessor import ThreatAssessor
from threat.pipeline import assess_frame


EGO_LENGTH_M = 4.90
EGO_WIDTH_M = 2.12
VEHICLE_LENGTH_M = 4.90
VEHICLE_WIDTH_M = 2.12
PED_SIZE_M = 0.50


@dataclass
class Body:
    track_id: str
    class_name: str
    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float
    v_lat_mps: float
    length_m: float
    width_m: float
    height_m: float


@dataclass
class SimResult:
    records: list[FrameRecord]
    events: list[dict]
    summary: object
    first_ttc_frame: int | None
    first_seen: dict[str, float]
    collided: bool
    collision_frame: int | None
    collision_relative_speed_mps: float | None
    end_reason: str
    samples: list | None = None


def simulate_scene(scene: GridScene, policy, actuator: BrakeActuator, *, occlusion: bool = False, timeout_s: float | None = None) -> SimResult:
    """命令从下一帧才改变自车。遮挡打开时，看不见的目标不会进入策略。"""
    limit_s = TIMEOUT_S if timeout_s is None else float(timeout_s)
    if limit_s <= 0.0:
        raise ValueError("timeout_s 必须大于 0")
    if hasattr(policy, "bind_scene"):
        policy.bind_scene(make_straight_road(120.0, 2.0), EGO_LENGTH_M, EGO_WIDTH_M, DT_S)
    bodies = _initial_bodies(scene)
    ego_x = EGO_S_M
    ego_y = 0.0
    ego_yaw = 0.0
    ego_speed = scene.ego_speed_mps
    prev_speed = None
    shown_yaw_rate = 0.0
    shown_alat = 0.0
    shown_steer = None
    triggered = False
    tracker = GroundTruthTracker()
    visibility = VisibilityModel(enabled=occlusion)
    assessor = ThreatAssessor()
    records: list[FrameRecord] = []
    samples: list[dict] = []
    events = [_spec_event(scene)]
    gaps = []
    collision_frame = None
    relative_speed = None
    end_reason = "timeout"
    last = int(math.floor(limit_s / DT_S + 1e-9))
    first_ttc = None
    for frame in range(last + 1):
        sim_time = frame * DT_S
        measured_a = 0.0 if prev_speed is None else (ego_speed - prev_speed) / DT_S
        ego = _ego_state(ego_x, ego_y, ego_yaw, ego_speed, measured_a, shown_alat, shown_yaw_rate, shown_steer)
        observations = [_truth(body, sim_time) for body in bodies]
        tracks, threat, request = assess_frame(
            timestamp_s=sim_time,
            ego=ego,
            observations=observations,
            ego_length_m=EGO_LENGTH_M,
            ego_width_m=EGO_WIDTH_M,
            cruise_speed_mps=scene.ego_speed_mps,
            tracker=tracker,
            visibility=visibility,
            assessor=assessor,
            policy=policy,
        )
        del tracks
        if first_ttc is None and threat.ttc_s is not None:
            first_ttc = frame
        request, actual = actuator.command(request)
        transition = getattr(policy, "last_transition", None)
        if transition is not None and getattr(transition, "reason", None):
            events.append(
                {
                    "event_type": "aeb_state",
                    "sim_time_s": sim_time,
                    "detail": {
                        "frame": frame,
                        "from": transition.previous,
                        "to": transition.current,
                        "reason": transition.reason,
                        "ttc_s": threat.ttc_s,
                    },
                }
            )
        gap = min(_gap(ego_x, ego_y, ego_yaw, body) for body in bodies)
        gaps.append(gap)
        overlapping = gap <= 1e-6
        record = FrameRecord(
            run_id=scene.scene_id,
            scene_id=scene.scene_id,
            strategy_name=policy.name,
            frame=frame,
            sim_time_s=sim_time,
            source_tag="gt",
            ego=ego,
            threat=threat,
            aeb_state=getattr(policy, "aeb_state", "IDLE"),
            aes_state=_aes_state(policy),
            trigger_reason=None if transition is None or transition.current == "EXIT" else transition.reason,
            exit_reason=None if transition is None or transition.current != "EXIT" else transition.reason,
            arbitration_reason=_report(policy, "arbitration_reason"),
            control_request=request,
            actual_control=actual,
            quality_flags=["kinematic_script"] if prev_speed is not None else ["kinematic_script", "decel_unobserved"],
            aes_direction=_report(policy, "aes_direction"),
            path_error_m=_report(policy, "path_error_m"),
            road_departure_m=_report(policy, "road_departure_m"),
            side_gap_m=_report(policy, "side_gap_m"),
            lateral_accel_mps2=_report(policy, "lateral_accel_mps2"),
        )
        records.append(record)
        samples.append(
            {
                "frame": frame,
                "sim_time_s": sim_time,
                "ego": {
                    "x_m": ego_x,
                    "y_m": ego_y,
                    "yaw_rad": ego_yaw,
                    "length_m": EGO_LENGTH_M,
                    "width_m": EGO_WIDTH_M,
                    "height_m": 1.51,
                },
                "bodies": [
                    {
                        "track_id": body.track_id,
                        "class_name": body.class_name,
                        "x_m": body.x_m,
                        "y_m": body.y_m,
                        "yaw_rad": body.yaw_rad,
                        "length_m": body.length_m,
                        "width_m": body.width_m,
                        "height_m": body.height_m,
                    }
                    for body in bodies
                ],
            }
        )
        if not triggered and _near(scene, bodies, ego_x, ego_y):
            triggered = True
        if overlapping:
            collision_frame = frame
            other = min(bodies, key=lambda body: _gap(ego_x, ego_y, ego_yaw, body))
            other_vx, other_vy = _world_velocity(other)
            ego_vx, ego_vy = forward_xy(ego_yaw)
            relative_speed = math.hypot(ego_speed * ego_vx - other_vx, ego_speed * ego_vy - other_vy)
            end_reason = "collision"
            events.append(
                {
                    "event_type": "collision",
                    "sim_time_s": sim_time,
                    "detail": {"frame": frame, "relative_speed_mps": relative_speed, "detector": "oriented_box"},
                }
            )
            break
        if frame > 0 and ego_speed <= 0.2 and all(body.speed_mps <= 0.2 and abs(body.v_lat_mps) <= 0.2 for body in bodies):
            end_reason = "stop"
            break
        if sim_time >= limit_s - 1e-9:
            break
        applied = 0.0 if actual.applied_decel_mps2 is None else float(actual.applied_decel_mps2)
        steer_norm = float(actual.steer)
        new_speed = max(0.0, ego_speed - applied * DT_S)
        wheelbase = float(getattr(policy, "wheelbase_m", 2.85))
        max_steer = float(getattr(policy, "max_steer_rad", 0.52))
        max_yaw_rate = float(getattr(policy, "max_yaw_rate_radps", 0.40))
        steer_rad = max(-max_steer, min(max_steer, steer_norm * max_steer))
        if abs(steer_norm) <= 1e-4 or new_speed < 0.3:
            yaw_rate = 0.0
            ego_x += math.cos(ego_yaw) * 0.5 * (ego_speed + new_speed) * DT_S
            ego_y += math.sin(ego_yaw) * 0.5 * (ego_speed + new_speed) * DT_S
        else:
            yaw_rate = new_speed * math.tan(steer_rad) / wheelbase
            yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate))
            step = 0.5 * (ego_speed + new_speed) * DT_S
            fx, fy = forward_xy(ego_yaw)
            ego_x += fx * step
            ego_y += fy * step
            ego_yaw = (ego_yaw + yaw_rate * DT_S + math.pi) % (2.0 * math.pi) - math.pi
        prev_speed = ego_speed
        ego_speed = new_speed
        shown_yaw_rate = yaw_rate
        shown_alat = prev_speed * yaw_rate
        shown_steer = steer_rad if abs(steer_norm) > 1e-4 else None
        _advance_targets(scene, bodies, triggered)
    if occlusion:
        seen_items = visibility.first_seen_s.items()
    else:
        seen_items = ((body.track_id, 0.0) for body in bodies)
    for track_id, seen in seen_items:
        events.append({"event_type": "first_seen", "sim_time_s": seen, "detail": {"track_id": track_id, "frame": int(round(seen / DT_S))}})
    collided = collision_frame is not None
    summary = make_summary(
        _summary_spec(scene),
        scene.scene_id,
        policy.name,
        records,
        gaps,
        collided,
        "collision" if collided else "avoided",
    )
    if not isinstance(summary, EpisodeSummary):
        raise TypeError("汇总必须是 EpisodeSummary")
    return SimResult(
        records=records,
        events=events,
        summary=summary,
        first_ttc_frame=first_ttc,
        first_seen=dict(visibility.first_seen_s),
        collided=collided,
        collision_frame=collision_frame,
        collision_relative_speed_mps=relative_speed,
        end_reason=end_reason,
        samples=samples,
    )


def _initial_bodies(scene: GridScene) -> list[Body]:
    origin = EGO_S_M + scene.distance_m
    if scene.template == "vehicle_cut_in":
        return [
            Body("cut_01", "vehicle", origin, 3.5, 0.0, max(0.0, scene.ego_speed_mps - 3.0), 0.0, VEHICLE_LENGTH_M, VEHICLE_WIDTH_M, 1.51)
        ]
    if scene.template == "occlusion_reveal":
        return [
            Body("blocker_01", "vehicle", origin - 3.0, 2.4, 0.0, 0.0, 0.0, 4.0, 2.2, 1.80),
            Body("ped_hidden", "pedestrian", origin, 2.6, -math.pi / 2.0, 0.85, 0.0, PED_SIZE_M, PED_SIZE_M, 1.80),
        ]
    if scene.template == "left_avoidable":
        return [Body("block_01", "vehicle", origin, 1.8, 0.0, 0.0, 0.0, VEHICLE_LENGTH_M, VEHICLE_WIDTH_M, 1.51)]
    if scene.template == "right_avoidable":
        return [Body("block_01", "vehicle", origin, -1.8, 0.0, 0.0, 0.0, VEHICLE_LENGTH_M, VEHICLE_WIDTH_M, 1.51)]
    if scene.template == "both_unavoidable":
        return [
            Body(f"block_{index}", "vehicle", origin, lateral, 0.0, 0.0, 0.0, VEHICLE_LENGTH_M, VEHICLE_WIDTH_M, 1.51)
            for index, lateral in enumerate((0.0, -3.5, 3.5))
        ]
    if scene.template == "parallel_vehicle":
        return [Body("side_01", "vehicle", EGO_S_M + scene.distance_m + 18.0, 3.6, 0.0, scene.ego_speed_mps, 0.0, VEHICLE_LENGTH_M, VEHICLE_WIDTH_M, 1.51)]
    raise ValueError(f"没有运动: {scene.template}")


def _advance_targets(scene: GridScene, bodies: list[Body], triggered: bool) -> None:
    """本帧已经记完。切入的横向运动从触发之后的下一帧才出现在观测里。"""
    if scene.template == "vehicle_cut_in":
        body = bodies[0]
        body.x_m += body.speed_mps * DT_S
        if triggered and body.y_m > 0.0:
            body.v_lat_mps = -2.0
            body.y_m = max(0.0, body.y_m + body.v_lat_mps * DT_S)
            if body.y_m <= 0.0:
                body.v_lat_mps = 0.0
        return
    if scene.template == "occlusion_reveal":
        bodies[1].y_m -= 0.85 * DT_S
        return
    if scene.template == "parallel_vehicle":
        bodies[0].x_m += bodies[0].speed_mps * DT_S


def _near(scene: GridScene, bodies: list[Body], ego_x: float, ego_y: float) -> bool:
    if scene.template == "vehicle_cut_in":
        return bodies[0].x_m - ego_x <= 9.5
    if scene.template == "parallel_vehicle":
        return False
    limit = scene.distance_m
    return any(math.hypot(body.x_m - ego_x, body.y_m - ego_y) <= limit for body in bodies)


def _gap(ego_x: float, ego_y: float, ego_yaw: float, body: Body) -> float:
    return surface_gap(
        ego_x,
        ego_y,
        ego_yaw,
        EGO_LENGTH_M,
        EGO_WIDTH_M,
        body.x_m,
        body.y_m,
        body.yaw_rad,
        body.length_m,
        body.width_m,
    )


def _truth(body: Body, sim_time: float) -> TargetTruth:
    return TargetTruth(
        track_id=body.track_id,
        class_name=body.class_name,
        x_m=body.x_m,
        y_m=body.y_m,
        z_m=0.0,
        yaw_rad=body.yaw_rad,
        v_long_mps=body.speed_mps,
        v_lat_mps=body.v_lat_mps,
        speed_mps=math.hypot(body.speed_mps, body.v_lat_mps),
        length_m=body.length_m,
        width_m=body.width_m,
        height_m=body.height_m,
        timestamp_s=sim_time,
    )


def _world_velocity(body: Body) -> tuple[float, float]:
    fx, fy = forward_xy(body.yaw_rad)
    rx, ry = right_xy(body.yaw_rad)
    return fx * body.speed_mps + rx * body.v_lat_mps, fy * body.speed_mps + ry * body.v_lat_mps


def _ego_state(x_m, y_m, yaw, speed, a_long, a_lat, yaw_rate, steer) -> EgoState:
    return EgoState(
        x_m=x_m,
        y_m=y_m,
        z_m=0.0,
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=yaw,
        v_long_mps=speed,
        v_lat_mps=0.0,
        v_up_mps=0.0,
        speed_mps=abs(speed),
        yaw_rate_radps=yaw_rate,
        a_long_mps2=a_long,
        a_lat_mps2=a_lat,
        steer_road_rad=steer,
    )


def _aes_state(policy) -> str:
    report = getattr(policy, "report", None)
    if report is not None and getattr(report, "aes_state", None):
        return report.aes_state
    return getattr(policy, "aes_state", AES_IDLE)


def _report(policy, name: str):
    report = getattr(policy, "report", None)
    if report is None:
        return None
    return getattr(report, name, None)


def _spec_event(scene: GridScene) -> dict:
    return {
        "event_type": "scenario_spec",
        "sim_time_s": 0.0,
        "detail": {
            "template": scene.template,
            "instance": scene.instance,
            "seed": scene.seed,
            "split": scene.split,
            "ego": {"s_m": EGO_S_M, "speed_mps": scene.ego_speed_mps},
            "target": {"gap_m": scene.distance_m},
            "trigger": {"distance_m": scene.distance_m},
            "end": {"hard_negative": scene.hard_negative, "timeout_s": TIMEOUT_S},
        },
    }


def _summary_spec(scene: GridScene):
    """make_summary 只用得到超时和停车速度。造一个最小替身，避免改场景加载器。"""

    class _Spec:
        timeout_s = TIMEOUT_S
        stop_speed_mps = 0.2
        scene_id = scene.scene_id

    return _Spec()


