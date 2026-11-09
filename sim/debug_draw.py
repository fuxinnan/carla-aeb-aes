"""CARLA 里给主威胁画框。只在 --debug-draw 时由场景循环调用。"""

from __future__ import annotations

import math

from schemas import ThreatResult


def draw_scenario_debug(
    world,
    ego_actor,
    target_actor,
    *,
    ego_length_m: float,
    ego_width_m: float,
    ego_height_m: float,
    target_length_m: float,
    target_width_m: float,
    target_height_m: float,
    threat: ThreatResult,
    life_time_s: float,
    aeb_state: str = "IDLE",
    notice: str | None = None,
) -> None:
    """自车框按 AEB 状态着色。状态切换时在框上写 ASCII 原因。主威胁头顶仍是 TTC。"""
    _draw_box(
        world,
        ego_actor,
        ego_length_m,
        ego_width_m,
        ego_height_m,
        _aeb_color(aeb_state),
        life_time_s,
        notice,
    )
    _draw_box(
        world,
        target_actor,
        target_length_m,
        target_width_m,
        target_height_m,
        _risk_color(threat),
        life_time_s,
        _ttc_text(threat),
    )


def draw_replay_frame(
    world,
    ego_actor,
    others: list,
    *,
    ego_length_m: float,
    ego_width_m: float,
    ego_height_m: float,
    threat: ThreatResult | None,
    life_time_s: float,
    aeb_state: str = "IDLE",
    notice: str | None = None,
) -> None:
    """自车按 AEB 状态着色。与当前威胁编号相同的目标写 TTC，其余目标用灰色。"""
    _draw_box(world, ego_actor, ego_length_m, ego_width_m, ego_height_m, _aeb_color(aeb_state), life_time_s, notice)
    threat_id = None if threat is None else threat.threat_id
    for actor, length_m, width_m, height_m, track_id in others:
        if threat is not None and track_id == threat_id:
            _draw_box(world, actor, length_m, width_m, height_m, _risk_color(threat), life_time_s, _ttc_text(threat))
        else:
            _draw_box(world, actor, length_m, width_m, height_m, _aeb_color("IDLE"), life_time_s, None)


def _draw_box(world, actor, length_m: float, width_m: float, height_m: float, color, life_time_s: float, text: str | None) -> None:
    import carla

    transform = actor.get_transform()
    center = transform.location
    box = carla.BoundingBox(
        center,
        carla.Vector3D(x=float(length_m) * 0.5, y=float(width_m) * 0.5, z=float(height_m) * 0.5),
    )
    world.debug.draw_box(box, transform.rotation, thickness=0.12, color=color, life_time=float(life_time_s))
    if not text:
        return
    world.debug.draw_string(
        center + carla.Location(z=float(height_m) * 0.5 + 0.6),
        text,
        False,
        color,
        float(life_time_s),
    )


def _aeb_color(state: str):
    import carla

    colors = {
        "IDLE": (158, 158, 158),
        "WARN": (249, 168, 37),
        "PARTIAL": (251, 140, 0),
        "FULL": (229, 57, 53),
        "EXIT": (30, 136, 229),
    }
    red, green, blue = colors.get(state, (33, 33, 33))
    return carla.Color(red, green, blue)


def _risk_color(threat: ThreatResult):
    import carla

    if threat.ttc_s is None or threat.invalid_reason:
        return carla.Color(158, 158, 158)
    if threat.brake_avoidable is False:
        return carla.Color(229, 57, 53)
    return carla.Color(249, 168, 37)


def _ttc_text(threat: ThreatResult) -> str:
    if threat.ttc_s is None or not math.isfinite(threat.ttc_s):
        return "TTC --"
    return f"TTC {threat.ttc_s:.2f}s"
