"""在线运动学闭环。自车速度由执行层减速度积分，目标仍走原来的脚本。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from aeb.base import Transition
from control.delay import brake_delay_frames
from perception.tracker import GroundTruthTracker, TargetTruth
from perception.visibility import VisibilityModel
from scenarios.geometry import RoadSample, forward_xy, surface_gap, wrap_pi
from scenarios.motion import ActorSample, ego_sample, target_sample
from scenarios.spec import ScenarioSpec
from schemas import (
    AES_IDLE,
    EgoState,
    EpisodeSummary,
    FrameRecord,
    Track,
)
from threat.assessor import ThreatAssessor
from threat.pipeline import assess_frame

_TARGETS = {
    "pedestrian_crossing": ("ped_01", "pedestrian"),
    "lead_vehicle_brake": ("lead_01", "vehicle"),
}


@dataclass
class ClosedStep:
    frame: int
    sim_time_s: float
    ego: ActorSample
    target: ActorSample
    distance_m: float
    gap_m: float
    trigger_edge: bool
    overlapping: bool
    record: FrameRecord
    tracks: list[Track]
    transition: Transition | None


@dataclass
class ClosedEpisode:
    steps: list[ClosedStep]
    end_reason: str
    collided: bool
    collision_frame: int | None
    trigger_frame: int | None
    collision_relative_speed_mps: float | None


def simulate_episode(spec: ScenarioSpec, road: list[RoadSample], policy, actuator, run_id: str) -> ClosedEpisode:
    """零制动且不转向时，自车弧长和触发帧与 rollout 一致。命令从下一帧才改变运动。"""
    if not road:
        raise ValueError("道路采样是空的")
    track_id, class_name = _TARGETS[spec.template]
    dt = spec.fixed_delta_seconds
    last = int(math.floor(spec.timeout_s / dt + 1e-9))
    ego_s = spec.ego_s_m
    ego_speed = spec.ego_speed_mps
    prev_speed = None
    trigger_time = None
    trigger_frame = None
    collision_frame = None
    relative_speed = None
    end_reason = "timeout"
    braking = False
    free = None
    shown_yaw_rate = 0.0
    shown_alat = 0.0
    shown_steer = None
    steps: list[ClosedStep] = []
    tracker = GroundTruthTracker()
    visibility = VisibilityModel()
    assessor = ThreatAssessor()
    if hasattr(policy, "bind_scene"):
        policy.bind_scene(road, spec.ego_length_m, spec.ego_width_m, dt)
    for frame in range(last + 1):
        sim_time = frame * dt
        target = target_sample(spec, road, sim_time, trigger_time)
        if free is None:
            measured_a = 0.0 if prev_speed is None else (ego_speed - prev_speed) / dt
            ego = ego_sample(road, ego_s, ego_speed, measured_a)
        else:
            free_x, free_y, free_yaw, free_speed = free
            measured_a = 0.0 if prev_speed is None else (free_speed - prev_speed) / dt
            ego = _free_ego(road, free_x, free_y, free_yaw, free_speed, measured_a)
            ego_speed = free_speed
        distance = math.hypot(ego.s_m - target.s_m, ego.d_m - target.d_m)
        gap = surface_gap(
            ego.x_m,
            ego.y_m,
            ego.yaw_rad,
            spec.ego_length_m,
            spec.ego_width_m,
            target.x_m,
            target.y_m,
            target.yaw_rad,
            spec.target_length_m,
            spec.target_width_m,
        )
        overlapping = gap <= 1e-6
        trigger_edge = trigger_time is None and distance <= spec.trigger_distance_m
        if trigger_edge:
            trigger_time = sim_time
            trigger_frame = frame
        ego_state = _ego_state(ego, yaw_rate=shown_yaw_rate, a_lat=shown_alat, steer=shown_steer)
        tracks, threat, request = assess_frame(
            timestamp_s=sim_time,
            ego=ego_state,
            observations=[_observation(spec, track_id, class_name, target, sim_time)],
            ego_length_m=spec.ego_length_m,
            ego_width_m=spec.ego_width_m,
            cruise_speed_mps=spec.ego_speed_mps,
            tracker=tracker,
            visibility=visibility,
            assessor=assessor,
            policy=policy,
        )
        request, actual = actuator.command(request)
        transition = policy.last_transition
        flags = ["kinematic_script"]
        if prev_speed is None:
            flags.append("decel_unobserved")
        trigger_reason = None
        exit_reason = None
        if transition is not None:
            if transition.current == "EXIT":
                exit_reason = transition.reason
            else:
                trigger_reason = transition.reason
        record = FrameRecord(
            run_id=run_id,
            scene_id=spec.scene_id,
            strategy_name=policy.name,
            frame=frame,
            sim_time_s=sim_time,
            source_tag="gt",
            ego=ego_state,
            threat=threat,
            aeb_state=policy.aeb_state,
            aes_state=_logged_aes_state(policy),
            trigger_reason=trigger_reason,
            exit_reason=exit_reason,
            arbitration_reason=_report_value(policy, "arbitration_reason"),
            control_request=request,
            actual_control=actual,
            quality_flags=flags,
            aes_direction=_report_value(policy, "aes_direction"),
            path_error_m=_report_value(policy, "path_error_m"),
            road_departure_m=_report_value(policy, "road_departure_m"),
            side_gap_m=_report_value(policy, "side_gap_m"),
            lateral_accel_mps2=_report_value(policy, "lateral_accel_mps2"),
        )
        steps.append(
            ClosedStep(
                frame=frame,
                sim_time_s=sim_time,
                ego=ego,
                target=target,
                distance_m=distance,
                gap_m=gap,
                trigger_edge=trigger_edge,
                overlapping=overlapping,
                record=record,
                tracks=tracks,
                transition=transition,
            )
        )
        if overlapping:
            collision_frame = frame
            relative_speed = math.hypot(ego.v_x_mps - target.v_x_mps, ego.v_y_mps - target.v_y_mps)
            end_reason = "collision"
            break
        if frame > 0 and ego.speed_mps <= spec.stop_speed_mps and target.speed_mps <= spec.stop_speed_mps:
            end_reason = "stop"
            break
        if sim_time >= spec.timeout_s - 1e-9:
            end_reason = "timeout"
            break
        applied = 0.0 if actual.applied_decel_mps2 is None else float(actual.applied_decel_mps2)
        steer_norm = float(actual.steer)
        if free is None and abs(steer_norm) <= 1e-4:
            if applied > 1e-9:
                braking = True
            if not braking:
                ego_s = spec.ego_s_m + spec.ego_speed_mps * ((frame + 1) * dt)
                prev_speed = ego_speed
                ego_speed = spec.ego_speed_mps
            else:
                new_speed = max(0.0, ego_speed - applied * dt)
                ego_s = ego_s + 0.5 * (ego_speed + new_speed) * dt
                prev_speed = ego_speed
                ego_speed = new_speed
            shown_yaw_rate = 0.0
            shown_alat = 0.0
            shown_steer = None
        else:
            if free is None:
                free = (ego.x_m, ego.y_m, ego.yaw_rad, ego_speed)
            free_x, free_y, free_yaw, free_speed = free
            wheelbase = float(getattr(policy, "wheelbase_m", 2.85))
            max_steer = float(getattr(policy, "max_steer_rad", 0.52))
            max_yaw_rate = float(getattr(policy, "max_yaw_rate_radps", 0.40))
            steer_rad = max(-max_steer, min(max_steer, steer_norm * max_steer))
            new_speed = max(0.0, free_speed - applied * dt)
            if free_speed < 0.3 or wheelbase <= 1e-6:
                yaw_rate = 0.0
            else:
                yaw_rate = free_speed * math.tan(steer_rad) / wheelbase
                yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate))
            distance = 0.5 * (free_speed + new_speed) * dt
            step_x, step_y = forward_xy(free_yaw)
            free = (
                free_x + step_x * distance,
                free_y + step_y * distance,
                wrap_pi(free_yaw + yaw_rate * dt),
                new_speed,
            )
            prev_speed = free_speed
            ego_speed = new_speed
            shown_yaw_rate = yaw_rate
            shown_alat = free_speed * yaw_rate
            shown_steer = steer_rad
    return ClosedEpisode(
        steps=steps,
        end_reason=end_reason,
        collided=collision_frame is not None,
        collision_frame=collision_frame,
        trigger_frame=trigger_frame,
        collision_relative_speed_mps=relative_speed,
    )


def make_summary(spec: ScenarioSpec, run_id: str, strategy_name: str, records: list[FrameRecord], gaps: list[float], collided: bool, outcome: str) -> EpisodeSummary:
    """汇总一次闭环。速度下降和延迟都从帧日志来，不从命令反推。"""
    speeds = [record.ego.speed_mps for record in records]
    decels = [-record.ego.a_long_mps2 for record in records]
    requested = [
        0.0 if record.control_request.target_decel_mps2 is None else record.control_request.target_decel_mps2
        for record in records
    ]
    measured = [max(0.0, -record.ego.a_long_mps2) for record in records]
    ttcs = [record.threat.ttc_s for record in records if record.threat.ttc_s is not None]
    trigger_index = next((index for index, record in enumerate(records) if record.aeb_state != "IDLE"), None)
    aes_index = next((index for index, record in enumerate(records) if record.aes_state != "IDLE"), None)
    if trigger_index is None:
        trigger_time = None
        speed_drop = None
    else:
        trigger_time = records[trigger_index].sim_time_s
        later = speeds[trigger_index:]
        speed_drop = max(0.0, speeds[trigger_index] - min(later))
    aes_time = None if aes_index is None else records[aes_index].sim_time_s
    first_times = [item for item in (trigger_time, aes_time) if item is not None]
    first_trigger = min(first_times) if first_times else None
    if decels and abs(max(decels)) < 1e-9:
        decels = [0.0]
    return EpisodeSummary(
        run_id=run_id,
        scene_id=spec.scene_id,
        strategy_name=strategy_name,
        outcome=outcome,
        collided=collided,
        min_distance_m=min(gaps) if gaps else None,
        min_ttc_s=min(ttcs) if ttcs else None,
        first_trigger_time_s=first_trigger,
        aeb_first_trigger_time_s=trigger_time,
        aes_first_trigger_time_s=aes_time,
        peak_decel_mps2=max(decels) if decels else None,
        speed_drop_mps=speed_drop,
        initial_speed_mps=speeds[0] if speeds else None,
        final_speed_mps=speeds[-1] if speeds else None,
        duration_s=records[-1].sim_time_s - records[0].sim_time_s,
        frame_count=len(records),
        invalid_frame_count=sum(record.threat.invalid_reason is not None for record in records),
        brake_delay_frames=brake_delay_frames(requested, measured),
    )


def _ego_state(sample: ActorSample, yaw_rate: float = 0.0, a_lat: float = 0.0, steer: float | None = None) -> EgoState:
    return EgoState(
        x_m=sample.x_m,
        y_m=sample.y_m,
        z_m=sample.z_m,
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=sample.yaw_rad,
        v_long_mps=sample.speed_mps,
        v_lat_mps=0.0,
        v_up_mps=0.0,
        speed_mps=abs(sample.speed_mps),
        yaw_rate_radps=yaw_rate,
        a_long_mps2=sample.a_long_mps2,
        a_lat_mps2=a_lat,
        steer_road_rad=steer,
    )


def _free_ego(road, x_m: float, y_m: float, yaw_rad: float, speed_mps: float, a_long_mps2: float) -> ActorSample:
    from aes.trajectory import project_xy

    s_m, d_m = project_xy(road, x_m, y_m)
    fx, fy = forward_xy(yaw_rad)
    return ActorSample(
        s_m=s_m,
        d_m=d_m,
        x_m=x_m,
        y_m=y_m,
        z_m=0.0,
        yaw_rad=yaw_rad,
        speed_mps=speed_mps,
        a_long_mps2=a_long_mps2,
        v_x_mps=fx * speed_mps,
        v_y_mps=fy * speed_mps,
    )


def _logged_aes_state(policy) -> str:
    report = getattr(policy, "report", None)
    if report is not None and getattr(report, "aes_state", None):
        return report.aes_state
    return getattr(policy, "aes_state", AES_IDLE)


def _report_value(policy, name: str):
    report = getattr(policy, "report", None)
    if report is None:
        return None
    return getattr(report, name)


def _observation(spec: ScenarioSpec, track_id: str, class_name: str, sample: ActorSample, sim_time_s: float) -> TargetTruth:
    return TargetTruth(
        track_id=track_id,
        class_name=class_name,
        x_m=sample.x_m,
        y_m=sample.y_m,
        z_m=sample.z_m,
        yaw_rad=sample.yaw_rad,
        v_long_mps=sample.speed_mps,
        v_lat_mps=0.0,
        speed_mps=abs(sample.speed_mps),
        length_m=spec.target_length_m,
        width_m=spec.target_width_m,
        height_m=spec.target_height_m,
        timestamp_s=sim_time_s,
    )
