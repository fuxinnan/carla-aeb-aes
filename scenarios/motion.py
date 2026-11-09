"""确定性运动学推演。触发和碰撞都由这套公式决定，不使用交通管理器。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from scenarios.geometry import RoadSample, forward_xy, pose_on_road, surface_gap, wrap_pi
from scenarios.spec import ScenarioSpec


@dataclass(frozen=True)
class ActorSample:
    s_m: float
    d_m: float
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    speed_mps: float
    a_long_mps2: float
    v_x_mps: float
    v_y_mps: float


@dataclass(frozen=True)
class StepSample:
    frame: int
    sim_time_s: float
    ego: ActorSample
    target: ActorSample
    distance_m: float
    surface_gap_m: float
    overlapping: bool
    trigger_edge: bool


@dataclass(frozen=True)
class Rollout:
    steps: tuple[StepSample, ...]
    trigger_frame: int | None
    collision_frame: int | None
    collision_relative_speed_mps: float | None
    end_reason: str


def rollout(spec: ScenarioSpec, road: list[RoadSample]) -> Rollout:
    """按固定时步推演。触发帧的目标仍然静止，下一帧才开始脚本动作。"""
    if not road:
        raise ValueError("道路采样是空的")
    timestep = spec.fixed_delta_seconds
    last_frame = int(math.floor(spec.timeout_s / timestep + 1e-9))
    trigger_time: float | None = None
    trigger_frame: int | None = None
    collision_frame: int | None = None
    relative_speed: float | None = None
    end_reason = "timeout"
    steps: list[StepSample] = []
    for frame in range(last_frame + 1):
        sim_time = frame * timestep
        ego = _ego_at(spec, road, sim_time)
        target = _target_at(spec, road, sim_time, trigger_time)
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
        steps.append(
            StepSample(
                frame=frame,
                sim_time_s=sim_time,
                ego=ego,
                target=target,
                distance_m=distance,
                surface_gap_m=gap,
                overlapping=overlapping,
                trigger_edge=trigger_edge,
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
    return Rollout(
        steps=tuple(steps),
        trigger_frame=trigger_frame,
        collision_frame=collision_frame,
        collision_relative_speed_mps=relative_speed,
        end_reason=end_reason,
    )


def _ego_at(spec: ScenarioSpec, road: list[RoadSample], sim_time_s: float) -> ActorSample:
    distance = spec.ego_s_m + spec.ego_speed_mps * sim_time_s
    return _body(road, distance, 0.0, 0.0, spec.ego_speed_mps, 0.0)


def target_sample(
    spec: ScenarioSpec,
    road: list[RoadSample],
    sim_time_s: float,
    trigger_time_s: float | None,
) -> ActorSample:
    """目标在这一帧的位姿。触发时刻本身仍是动作前的状态。"""
    return _target_at(spec, road, sim_time_s, trigger_time_s)


def ego_sample(road: list[RoadSample], s_m: float, speed_mps: float, a_long_mps2: float) -> ActorSample:
    """自车沿车道中心的运动学采样。a_long_mps2 是这一帧的纵向加速度，向前为正。"""
    return _body(road, s_m, 0.0, 0.0, speed_mps, a_long_mps2)


def _target_at(
    spec: ScenarioSpec,
    road: list[RoadSample],
    sim_time_s: float,
    trigger_time_s: float | None,
) -> ActorSample:
    if spec.template == "pedestrian_crossing":
        return _pedestrian_at(spec, road, sim_time_s, trigger_time_s)
    if spec.template == "lead_vehicle_brake":
        return _lead_at(spec, road, sim_time_s, trigger_time_s)
    raise ValueError(f"没有运动脚本: {spec.template}")


def _pedestrian_at(
    spec: ScenarioSpec,
    road: list[RoadSample],
    sim_time_s: float,
    trigger_time_s: float | None,
) -> ActorSample:
    lateral0, direction = _pedestrian_side(spec)
    if trigger_time_s is None or sim_time_s <= trigger_time_s + 1e-12:
        lateral = lateral0
        speed = 0.0
    else:
        elapsed = sim_time_s - trigger_time_s
        lateral = lateral0 + direction * spec.target_speed_mps * elapsed
        speed = spec.target_speed_mps
    return _body(road, float(spec.crossing_s_m), lateral, direction * math.pi / 2.0, speed, 0.0)


def _lead_at(
    spec: ScenarioSpec,
    road: list[RoadSample],
    sim_time_s: float,
    trigger_time_s: float | None,
) -> ActorSample:
    origin = spec.ego_s_m + float(spec.target_gap_m)
    speed0 = spec.target_speed_mps
    decel = float(spec.target_decel_mps2)
    if trigger_time_s is None or sim_time_s <= trigger_time_s + 1e-12:
        return _body(road, origin + speed0 * sim_time_s, 0.0, 0.0, speed0, 0.0)
    distance_at_trigger = origin + speed0 * trigger_time_s
    elapsed = sim_time_s - trigger_time_s
    if decel <= 1e-9:
        return _body(road, distance_at_trigger + speed0 * elapsed, 0.0, 0.0, speed0, 0.0)
    stop_time = speed0 / decel if speed0 > 0.0 else 0.0
    if elapsed >= stop_time:
        stop_distance = distance_at_trigger + speed0 * stop_time - 0.5 * decel * stop_time * stop_time
        return _body(road, stop_distance, 0.0, 0.0, 0.0, 0.0)
    distance = distance_at_trigger + speed0 * elapsed - 0.5 * decel * elapsed * elapsed
    speed = speed0 - decel * elapsed
    return _body(road, distance, 0.0, 0.0, speed, -decel)


def _pedestrian_side(spec: ScenarioSpec) -> tuple[float, float]:
    lateral = float(spec.target_lateral_m)
    if spec.pedestrian_side == "left":
        return -lateral, 1.0
    if spec.pedestrian_side == "right":
        return lateral, -1.0
    raise ValueError("行人模板缺少 left 或 right")


def _body(
    road: list[RoadSample],
    s_m: float,
    d_m: float,
    yaw_offset_rad: float,
    speed_mps: float,
    a_long_mps2: float,
) -> ActorSample:
    x_m, y_m, z_m, road_yaw = pose_on_road(road, s_m, d_m)
    yaw = wrap_pi(road_yaw + yaw_offset_rad)
    fx, fy = forward_xy(yaw)
    return ActorSample(
        s_m=s_m,
        d_m=d_m,
        x_m=x_m,
        y_m=y_m,
        z_m=z_m,
        yaw_rad=yaw,
        speed_mps=speed_mps,
        a_long_mps2=a_long_mps2,
        v_x_mps=fx * speed_mps,
        v_y_mps=fy * speed_mps,
    )
