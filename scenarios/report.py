"""重复实验结果是否稳定。不连接仿真器。"""

from __future__ import annotations


SPAWN_TOLERANCE_M = 1e-3
YAW_TOLERANCE_RAD = 1e-4
SPEED_TOLERANCE_MPS = 1e-3


def assess_repeats(results: list[dict]) -> dict:
    """同一 YAML、同一种子：生成位置、触发帧、是否碰撞、相对速度都要齐。"""
    if not results:
        raise ValueError("没有重复运行")
    first = results[0]
    spawn_deviation, spawn_yaw = _spawn_deviation(results, "target_spawn")
    ego_deviation, ego_yaw = _spawn_deviation(results, "ego_spawn")
    triggers = [item["trigger_frame"] for item in results]
    collided = [bool(item["collided"]) for item in results]
    speeds = [item["collision_relative_speed_mps"] for item in results]
    numeric_speeds = [float(speed) for speed in speeds if speed is not None]
    speed_range = None if not numeric_speeds else max(numeric_speeds) - min(numeric_speeds)
    if collided[0]:
        speeds_match = all(speed is not None for speed in speeds) and speed_range is not None and speed_range <= SPEED_TOLERANCE_MPS
    else:
        speeds_match = all(speed is None for speed in speeds)
    stable = (
        spawn_deviation <= SPAWN_TOLERANCE_M
        and ego_deviation <= SPAWN_TOLERANCE_M
        and spawn_yaw <= YAW_TOLERANCE_RAD
        and ego_yaw <= YAW_TOLERANCE_RAD
        and len(set(triggers)) == 1
        and len(set(collided)) == 1
        and speeds_match
    )
    return {
        "stable": stable,
        "repeats": len(results),
        "target_spawn_deviation_m": spawn_deviation,
        "ego_spawn_deviation_m": ego_deviation,
        "trigger_frames": triggers,
        "collided": collided,
        "collision_relative_speed_mps": speeds,
        "speed_range_mps": speed_range,
        "target_spawn": dict(first["target_spawn"]),
        "ego_spawn": dict(first["ego_spawn"]),
    }


def format_repeat_report(scene_id: str, report: dict) -> str:
    speeds = report["collision_relative_speed_mps"]
    rendered = ", ".join("null" if speed is None else f"{float(speed):.6f}" for speed in speeds)
    span = report["speed_range_mps"]
    span_text = "null" if span is None else f"{float(span):.6f}"
    lines = [
        f"scene={scene_id}",
        f"repeats={report['repeats']}",
        f"stable={int(bool(report['stable']))}",
        f"target_spawn_deviation_m={report['target_spawn_deviation_m']:.6f}",
        f"trigger_frames={report['trigger_frames']}",
        f"collided={report['collided']}",
        f"collision_relative_speed_mps=[{rendered}]",
        f"speed_range_mps={span_text}",
        f"pose_ok={int(bool(report.get('pose_ok', True)))}",
    ]
    return "\n".join(lines)


def _spawn_deviation(results: list[dict], key: str) -> tuple[float, float]:
    first = results[0][key]
    position = 0.0
    yaw = 0.0
    for item in results:
        pose = item[key]
        for axis in ("x_m", "y_m", "z_m"):
            position = max(position, abs(float(pose[axis]) - float(first[axis])))
        yaw = max(yaw, abs(_yaw_delta(float(pose["yaw_rad"]), float(first["yaw_rad"]))))
    return position, yaw


def _yaw_delta(left: float, right: float) -> float:
    return (left - right + 3.141592653589793) % (2.0 * 3.141592653589793) - 3.141592653589793
