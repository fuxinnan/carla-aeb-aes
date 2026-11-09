"""场景模板和参数实例。模板规定脚本结构，实例只填数值。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


TEMPLATES = Path(__file__).resolve().parent / "templates"

_TOP_KEYS = {
    "template",
    "instance",
    "seed",
    "ego",
    "target",
    "trigger",
    "end",
    "map",
    "fixed_delta_seconds",
    "render",
    "host",
    "port",
    "tm_port",
}
_EGO_KEYS = {"speed_mps"}
_TRIGGER_KEYS = {"distance_m"}
_END_KEYS = {"hard_negative", "timeout_s", "stop_speed_mps"}
_PED_TARGET_KEYS = {"speed_mps", "lateral_m"}
_LEAD_TARGET_KEYS = {"speed_mps", "decel_mps2", "gap_m"}
_TARGET_BEHAVIORS = {"fixed_speed_walk": "pedestrian_crossing", "fixed_deceleration": "lead_vehicle_brake"}


@dataclass(frozen=True)
class ScenarioSpec:
    """一次可复现实验的全部场景参数。runtime 只在生成 actor 时临时绑上。"""

    template: str
    instance: str
    seed: int
    map_name: str
    fixed_delta_seconds: float
    timeout_s: float
    stop_speed_mps: float
    hard_negative: bool
    render: bool
    host: str
    port: int
    tm_port: int
    ego_speed_mps: float
    ego_s_m: float
    ego_blueprint: str
    ego_length_m: float
    ego_width_m: float
    ego_height_m: float
    target_speed_mps: float
    target_blueprint: str
    target_length_m: float
    target_width_m: float
    target_height_m: float
    target_decel_mps2: float | None
    target_gap_m: float | None
    target_lateral_m: float | None
    pedestrian_side: str | None
    crossing_s_m: float | None
    trigger_type: str
    trigger_distance_m: float
    straight_length_m: float
    sample_step_m: float
    execution: str
    ego_behavior: str
    target_behavior: str
    config_path: Path
    template_path: Path
    runtime: tuple | None = None

    @property
    def scene_id(self) -> str:
        return f"{self.template}_{self.instance}"

    @property
    def target_class(self) -> str:
        if self.template == "pedestrian_crossing":
            return "pedestrian"
        return "vehicle"

    def key_fields(self) -> dict:
        """写进 manifest 和 events 的字段。不含到仿真器的连接。"""
        return {
            "template": self.template,
            "instance": self.instance,
            "seed": self.seed,
            "map": self.map_name,
            "fixed_delta_seconds": self.fixed_delta_seconds,
            "execution": self.execution,
            "behavior": {"ego": self.ego_behavior, "target": self.target_behavior},
            "ego": {
                "blueprint": self.ego_blueprint,
                "speed_mps": self.ego_speed_mps,
                "s_m": self.ego_s_m,
                "length_m": self.ego_length_m,
                "width_m": self.ego_width_m,
                "height_m": self.ego_height_m,
            },
            "target": {
                "blueprint": self.target_blueprint,
                "class_name": self.target_class,
                "speed_mps": self.target_speed_mps,
                "decel_mps2": self.target_decel_mps2,
                "gap_m": self.target_gap_m,
                "lateral_m": self.target_lateral_m,
                "side": self.pedestrian_side,
                "crossing_s_m": self.crossing_s_m,
                "length_m": self.target_length_m,
                "width_m": self.target_width_m,
                "height_m": self.target_height_m,
            },
            "trigger": {"type": self.trigger_type, "distance_m": self.trigger_distance_m},
            "road": {
                "source": "carla_lane",
                "straight_length_m": self.straight_length_m,
                "sample_step_m": self.sample_step_m,
            },
            "end": {
                "conditions": ["collision", "stop", "timeout"],
                "timeout_s": self.timeout_s,
                "stop_speed_mps": self.stop_speed_mps,
                "hard_negative": self.hard_negative,
            },
        }


def load_spec(path: str | Path) -> ScenarioSpec:
    """读参数实例，并按同名模板校验。实例不能改脚本，也不能交给交通管理器。"""
    config_path = Path(path)
    raw = _read_mapping(config_path)
    unknown = sorted(set(raw) - _TOP_KEYS)
    if unknown:
        raise ValueError(f"实例含未知字段: {unknown}")
    if "behavior" in raw or "execution" in raw:
        raise ValueError("参数实例不能改写行为。目标用确定性脚本，不使用交通管理器。")
    template_name = _as_str(_must(raw, "template"), "template")
    template_path = TEMPLATES / f"{template_name}.yaml"
    if not template_path.is_file():
        raise ValueError(f"没有模板 {template_name}: {template_path}")
    template = _read_mapping(template_path)
    if template.get("template") != template_name:
        raise ValueError(f"{template_path.name} 的 template 字段必须是 {template_name}")
    behavior = template.get("behavior") or {}
    target_behavior = str(behavior.get("target", ""))
    if target_behavior in {"traffic_manager", "autopilot", "walker_ai"}:
        raise ValueError("模板不能把目标交给交通管理器或行人导航")
    if template.get("execution") != "kinematic":
        raise ValueError("这一轮的模板 execution 必须是 kinematic")
    if _TARGET_BEHAVIORS.get(target_behavior) != template_name:
        raise ValueError(f"{template_name} 的目标行为 {target_behavior} 不匹配")
    for dotted in template.get("required_instance") or []:
        _lookup(raw, str(dotted))
    _reject_nested(raw.get("ego") or {}, _EGO_KEYS, "ego")
    _reject_nested(raw.get("trigger") or {}, _TRIGGER_KEYS, "trigger")
    _reject_nested(raw.get("end") or {}, _END_KEYS, "end")
    if template_name == "pedestrian_crossing":
        _reject_nested(raw.get("target") or {}, _PED_TARGET_KEYS, "target")
    elif template_name == "lead_vehicle_brake":
        _reject_nested(raw.get("target") or {}, _LEAD_TARGET_KEYS, "target")
    else:
        raise ValueError(f"不支持的模板 {template_name}")
    _check_bounds(template, raw)
    return _assemble(template, raw, config_path, template_path)


def _assemble(template: dict, raw: dict, config_path: Path, template_path: Path) -> ScenarioSpec:
    template_name = str(template["template"])
    ego_box = _box(template, "ego_box")
    target_box = _box(template, "target_box")
    side = template.get("pedestrian_side")
    if template_name == "pedestrian_crossing" and side not in {"left", "right"}:
        raise ValueError("行人模板的 pedestrian_side 必须是 left 或 right")
    trigger = template.get("trigger") or {}
    if trigger.get("type") != "distance_to_target":
        raise ValueError("触发类型必须是 distance_to_target")
    timeout_s = _pick_float(raw, template, "end.timeout_s", "timeout_s", template.get("timeout_s"))
    stop_speed = _pick_float(raw, template, "end.stop_speed_mps", "stop_speed_mps", template.get("stop_speed_mps"))
    if timeout_s <= 0.0 or stop_speed < 0.0:
        raise ValueError("超时必须大于 0，停车速度不能为负")
    timestep = _pick_float(raw, template, "fixed_delta_seconds", "fixed_delta_seconds", None)
    if timestep <= 0.0:
        raise ValueError("fixed_delta_seconds 必须大于 0")
    return ScenarioSpec(
        template=template_name,
        instance=_as_str(_lookup(raw, "instance"), "instance"),
        seed=_as_int(_lookup(raw, "seed"), "seed"),
        map_name=_as_str(_pick(raw, "map", template.get("map")), "map"),
        fixed_delta_seconds=timestep,
        timeout_s=timeout_s,
        stop_speed_mps=stop_speed,
        hard_negative=_as_bool(_lookup(raw, "end.hard_negative"), "end.hard_negative"),
        render=_as_bool(_pick(raw, "render", template.get("render", False)), "render"),
        host=_as_str(_pick(raw, "host", template.get("host", "localhost")), "host"),
        port=_as_int(_pick(raw, "port", template.get("port", 2000)), "port"),
        tm_port=_as_int(_pick(raw, "tm_port", template.get("tm_port", 8000)), "tm_port"),
        ego_speed_mps=_as_float(_lookup(raw, "ego.speed_mps"), "ego.speed_mps"),
        ego_s_m=_as_float(template["ego_s_m"], "ego_s_m"),
        ego_blueprint=_as_str(template["ego_blueprint"], "ego_blueprint"),
        ego_length_m=ego_box[0],
        ego_width_m=ego_box[1],
        ego_height_m=ego_box[2],
        target_speed_mps=_as_float(_lookup(raw, "target.speed_mps"), "target.speed_mps"),
        target_blueprint=_as_str(template["target_blueprint"], "target_blueprint"),
        target_length_m=target_box[0],
        target_width_m=target_box[1],
        target_height_m=target_box[2],
        target_decel_mps2=_optional_float(raw, "target.decel_mps2"),
        target_gap_m=_optional_float(raw, "target.gap_m"),
        target_lateral_m=_optional_float(raw, "target.lateral_m"),
        pedestrian_side=None if template_name != "pedestrian_crossing" else str(side),
        crossing_s_m=None if template_name != "pedestrian_crossing" else _as_float(template["crossing_s_m"], "crossing_s_m"),
        trigger_type="distance_to_target",
        trigger_distance_m=_as_float(_lookup(raw, "trigger.distance_m"), "trigger.distance_m"),
        straight_length_m=_as_float(template["straight_length_m"], "straight_length_m"),
        sample_step_m=_as_float(template["sample_step_m"], "sample_step_m"),
        execution="kinematic",
        ego_behavior=str((template.get("behavior") or {})["ego"]),
        target_behavior=str((template.get("behavior") or {})["target"]),
        config_path=config_path,
        template_path=template_path,
    )


def _check_bounds(template: dict, raw: dict) -> None:
    for dotted, limit in (template.get("bounds") or {}).items():
        try:
            value = _lookup(raw, str(dotted))
        except ValueError:
            continue
        number = _as_float(value, str(dotted))
        low = float(limit["min"])
        high = float(limit["max"])
        if number < low or number > high:
            raise ValueError(f"{dotted}={number} 超出范围 {low}..{high}")


def _box(template: dict, name: str) -> tuple[float, float, float]:
    raw = template.get(name)
    if not isinstance(raw, dict):
        raise ValueError(f"模板缺少 {name}")
    length = _as_float(raw.get("length_m"), f"{name}.length_m")
    width = _as_float(raw.get("width_m"), f"{name}.width_m")
    height = _as_float(raw.get("height_m"), f"{name}.height_m")
    if min(length, width, height) <= 0.0:
        raise ValueError(f"{name} 的长宽高必须大于 0")
    return length, width, height


def _pick_float(raw: dict, template: dict, instance_path: str, template_key: str, fallback) -> float:
    try:
        return _as_float(_lookup(raw, instance_path), instance_path)
    except ValueError as exc:
        if "缺少字段" not in str(exc):
            raise
    if template_key in template:
        return _as_float(template[template_key], template_key)
    if fallback is not None:
        return _as_float(fallback, template_key)
    raise ValueError(f"缺少 {instance_path}")


def _optional_float(raw: dict, dotted: str) -> float | None:
    try:
        return _as_float(_lookup(raw, dotted), dotted)
    except ValueError as exc:
        if "缺少字段" in str(exc):
            return None
        raise


def _pick(raw: dict, key: str, fallback):
    if key in raw:
        return raw[key]
    return fallback


def _reject_nested(raw: dict, allowed: set[str], name: str) -> None:
    if raw is None:
        return
    if not isinstance(raw, dict):
        raise ValueError(f"{name} 必须是映射")
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError(f"{name} 含本模板不使用的字段: {unknown}")


def _read_mapping(path: Path) -> dict:
    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError(f"YAML 必须是映射: {path}")
    return loaded


def _lookup(data: dict, dotted: str):
    current = data
    for part in dotted.split("."):
        if not isinstance(current, dict) or part not in current:
            raise ValueError(f"缺少字段: {dotted}")
        current = current[part]
    return current


def _must(data: dict, key: str):
    if key not in data:
        raise ValueError(f"缺少字段: {key}")
    return data[key]


def _as_str(value, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} 必须是非空字符串")
    return value.strip()


def _as_bool(value, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} 必须是 true 或 false")
    return value


def _as_int(value, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} 必须是整数")
    return value


def _as_float(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是数值")
    return float(value)
