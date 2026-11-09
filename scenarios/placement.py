"""在已加载的地图上找一段同车道，供场景沿弧长摆放。"""

from __future__ import annotations

import math

import carla

from scenarios.geometry import RoadSample


def find_straight_road(world, length_m: float, step_m: float) -> tuple[list[RoadSample], int]:
    """按出生点顺序找第一段够长、没有路口、单步转角不大的车道。"""
    if length_m <= 0.0 or step_m <= 0.0:
        raise ValueError("道路长度和采样间隔必须大于 0")
    carla_map = world.get_map()
    spawns = list(carla_map.get_spawn_points())
    if not spawns:
        raise RuntimeError("地图没有出生点")
    steps = int(math.ceil(length_m / step_m))
    for index, spawn in enumerate(spawns):
        waypoint = carla_map.get_waypoint(
            spawn.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        chain = _follow(waypoint, steps, step_m)
        if chain is None:
            continue
        return _samples(chain), index
    raise RuntimeError(
        f"在 {len(spawns)} 个出生点里没有找到 {length_m:.0f} m 的同车道。"
        "把模板里的 straight_length_m 改小，或换一条有长直道的地图。"
    )


def _follow(waypoint, steps: int, step_m: float):
    if waypoint is None or bool(waypoint.is_junction):
        return None
    chain = [waypoint]
    current = waypoint
    for _ in range(steps):
        nxt = current.next(float(step_m))
        if not nxt:
            return None
        current = nxt[0]
        if current.road_id != waypoint.road_id or current.lane_id != waypoint.lane_id:
            return None
        if bool(current.is_junction):
            return None
        if abs(_deg_diff(current.transform.rotation.yaw, chain[-1].transform.rotation.yaw)) > 12.0:
            return None
        chain.append(current)
    if len(chain) != steps + 1:
        return None
    return chain


def _samples(chain) -> list[RoadSample]:
    samples = []
    distance = 0.0
    previous = None
    for waypoint in chain:
        location = waypoint.transform.location
        if previous is not None:
            distance += math.hypot(location.x - previous.x, location.y - previous.y)
        samples.append(
            RoadSample(
                s_m=distance,
                x_m=float(location.x),
                y_m=float(location.y),
                z_m=float(location.z),
                yaw_rad=math.radians(float(waypoint.transform.rotation.yaw)),
                lane_half_width_m=float(waypoint.lane_width) * 0.5,
            )
        )
        previous = location
    return samples


def _deg_diff(left: float, right: float) -> float:
    return (left - right + 180.0) % 360.0 - 180.0
