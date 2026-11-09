"""按模板生成自车和唯一目标。目标不进交通管理器。"""

from __future__ import annotations

import math
from dataclasses import dataclass

import carla

from scenarios.motion import ActorSample, rollout
from scenarios.spec import ScenarioSpec


@dataclass
class SpawnedActors:
    ego: object
    target: object
    track_id: str
    class_name: str


def pedestrian_crossing(spec: ScenarioSpec) -> SpawnedActors:
    """行人横穿：路侧静止，触发后以固定速度走过自车道。"""
    session, road = _runtime(spec)
    step = rollout(spec, road).steps[0]
    ego = _spawn(session, spec.ego_blueprint, step.ego, "ego")
    target = _spawn(session, spec.target_blueprint, step.target, "pedestrian")
    return SpawnedActors(ego=ego, target=target, track_id="ped_01", class_name="pedestrian")


def lead_vehicle_brake(spec: ScenarioSpec) -> SpawnedActors:
    """前车制动：先匀速，触发后按固定减速度减速。不用自动驾驶。"""
    session, road = _runtime(spec)
    step = rollout(spec, road).steps[0]
    ego = _spawn(session, spec.ego_blueprint, step.ego, "ego")
    target = _spawn(session, spec.target_blueprint, step.target, "lead")
    return SpawnedActors(ego=ego, target=target, track_id="lead_01", class_name="vehicle")


def _runtime(spec: ScenarioSpec):
    if spec.runtime is None or len(spec.runtime) != 2:
        raise RuntimeError("调用 spawn 前要把会话和道路采样写进 spec.runtime")
    return spec.runtime


def _spawn(session, blueprint_id: str, sample: ActorSample, role: str):
    world = session.world
    blueprint = world.get_blueprint_library().find(blueprint_id)
    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute("role_name", role)
    if blueprint.has_attribute("is_invincible"):
        blueprint.set_attribute("is_invincible", "true")
    actor = _try_spawn(world, blueprint, sample, z_lift=0.0)
    if actor is None:
        actor = _try_spawn(world, blueprint, sample, z_lift=0.4)
    if actor is None:
        raise RuntimeError(
            f"无法生成 {blueprint_id}，位置 ({sample.x_m:.2f}, {sample.y_m:.2f}, {sample.z_m:.2f})"
        )
    actor.set_simulate_physics(False)
    actor.set_target_velocity(carla.Vector3D(float(sample.v_x_mps), float(sample.v_y_mps), 0.0))
    session.track_actor(actor)
    return actor


def _try_spawn(world, blueprint, sample: ActorSample, z_lift: float):
    transform = carla.Transform(
        carla.Location(x=float(sample.x_m), y=float(sample.y_m), z=float(sample.z_m) + z_lift),
        carla.Rotation(pitch=0.0, yaw=math.degrees(float(sample.yaw_rad)), roll=0.0),
    )
    return world.try_spawn_actor(blueprint, transform)
