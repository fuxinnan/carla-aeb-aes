"""场景模板。这一轮只有行人横穿和前车制动。

spawn 会连接 CARLA，所以放在函数里按需导入，避免只读规格时把 carla 引进来。
"""

from scenarios.spec import ScenarioSpec, load_spec

__all__ = [
    "ScenarioSpec",
    "lead_vehicle_brake",
    "load_spec",
    "pedestrian_crossing",
]


def __getattr__(name: str):
    if name in {"pedestrian_crossing", "lead_vehicle_brake"}:
        from scenarios.spawn import lead_vehicle_brake, pedestrian_crossing

        return {"pedestrian_crossing": pedestrian_crossing, "lead_vehicle_brake": lead_vehicle_brake}[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
