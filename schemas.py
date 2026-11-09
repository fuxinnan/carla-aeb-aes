"""闭环评测的公共数据结构。本模块不连接仿真器。

坐标系（全项目固定）：
- 世界系：左手系，单位米。X 向前，Y 向右，Z 向上。
  航向绕 Z，0 沿 +X，俯视顺时针为正。与 CARLA 世界系同向，入库前把度换算成弧度。
- 车体系：原点在车辆包围盒中心。X 沿车头，Y 向右，Z 向上。
- 相对量：写在自车车体系下。纵向向前为正，横向向右为正。
- 转角除特别声明外一律用弧度。转向控制量 steer 是无量纲 -1~1，向右为正。
"""

from __future__ import annotations

import json
import types
from dataclasses import MISSING, dataclass, fields, is_dataclass
from typing import Any, Union, get_args, get_origin, get_type_hints


SCHEMA_VERSION = 1

AEB_IDLE = "IDLE"
AEB_WARN = "WARN"
AEB_PARTIAL = "PARTIAL"
AEB_FULL = "FULL"
AEB_EXIT = "EXIT"
AEB_STATES = (AEB_IDLE, AEB_WARN, AEB_PARTIAL, AEB_FULL, AEB_EXIT)

AES_IDLE = "IDLE"
AES_ACTIVE = "ACTIVE"
AES_EXIT = "EXIT"
AES_STATES = (AES_IDLE, AES_ACTIVE, AES_EXIT)


@dataclass(frozen=True)
class EgoState:
    """自车运动状态。位置参考点是车辆包围盒中心。"""

    x_m: float  # 单位 m。世界系 X，向前为正。
    y_m: float  # 单位 m。世界系 Y，向右为正。
    z_m: float  # 单位 m。世界系 Z，向上为正。
    roll_rad: float  # 单位 rad。世界系绕 X，右侧下沉为正。
    pitch_rad: float  # 单位 rad。世界系绕 Y，抬头为正。
    yaw_rad: float  # 单位 rad。世界系绕 Z，0 沿 +X，俯视顺时针为正。
    v_long_mps: float  # 单位 m/s。车体系纵向速度，车头方向为正。
    v_lat_mps: float  # 单位 m/s。车体系横向速度，向右为正。
    v_up_mps: float  # 单位 m/s。车体系垂向速度，向上为正。
    speed_mps: float  # 单位 m/s。水平速率，非负标量，无方向。
    yaw_rate_radps: float  # 单位 rad/s。绕世界系 Z，与 yaw_rad 同号。
    a_long_mps2: float  # 单位 m/s^2。车体系纵向加速度，加速为正，制动为负。
    a_lat_mps2: float  # 单位 m/s^2。车体系横向加速度，向右为正。
    steer_road_rad: float | None  # 单位 rad。前轮转角，向右为正。未知为 None。


@dataclass(frozen=True)
class Track:
    """一条目标航迹。可以是真值，也可以是感知输出，靠 source 区分。"""

    track_id: str  # 无单位。场景内稳定的航迹编号，无坐标系。
    source: str  # 无单位。来源标签，如 gt 或 perception，无坐标系。
    class_name: str  # 无单位。类别名，如 vehicle、pedestrian，无坐标系。
    x_m: float  # 单位 m。世界系 X，向前为正。参考点是目标包围盒中心。
    y_m: float  # 单位 m。世界系 Y，向右为正。
    z_m: float  # 单位 m。世界系 Z，向上为正。
    yaw_rad: float  # 单位 rad。世界系航向，0 沿 +X，俯视顺时针为正。
    v_long_mps: float  # 单位 m/s。目标车体系纵向速度，车头方向为正。
    v_lat_mps: float  # 单位 m/s。目标车体系横向速度，向右为正。
    speed_mps: float  # 单位 m/s。目标水平速率，非负标量，无方向。
    length_m: float  # 单位 m。包围盒长度，沿目标车头，无方向坐标系。
    width_m: float  # 单位 m。包围盒宽度，沿目标车体横向。
    height_m: float  # 单位 m。包围盒高度，沿世界系 Z。
    rel_x_m: float  # 单位 m。自车车体系纵向距离，目标在车头前方为正。
    rel_y_m: float  # 单位 m。自车车体系横向距离，目标在右侧为正。
    rel_v_long_mps: float  # 单位 m/s。自车车体系下目标减自车的纵向速度，目标更快为正。
    rel_v_lat_mps: float  # 单位 m/s。自车车体系下目标减自车的横向速度，向右为正。
    in_path: bool | None  # 无单位。目标当前是否压在自车车宽走廊上。未知为 None。
    timestamp_s: float | None = None  # 单位 s。这条航迹对齐的仿真时间。旧日志没有该字段时为 None。
    first_seen_s: float | None = None  # 单位 s。遮挡模型第一次把这条航迹交给策略的仿真时间。还没看见，或本轮没启用遮挡时为 None。


@dataclass(frozen=True)
class ThreatResult:
    """当前最危险目标的威胁量。ttc 或预测最小间距无效时必须写 invalid_reason。"""

    ttc_s: float | None  # 单位 s。预测碰撞时间。无法计算时为 None。
    pred_min_distance_m: float | None  # 单位 m。预测最小表面间距，非负标量。无法计算时为 None。
    invalid_reason: str | None  # 无单位。上面两项有任一为 None 时写原因；都有效时必须是 None。
    threat_id: str | None  # 无单位。对应 Track.track_id。没有威胁目标时为 None。
    in_path: bool | None  # 无单位。该威胁是否在自车路径上。未知为 None。
    closing_speed_mps: float | None  # 单位 m/s。纵向靠近速度，相互靠近为正，等于 -rel_v_long。未知为 None。
    overlap_ratio: float | None  # 无单位。路径重叠比例，0~1。未知为 None。
    conflict_x_m: float | None = None  # 单位 m。世界系冲突点 X。没有冲突时刻时为 None。
    conflict_y_m: float | None = None  # 单位 m。世界系冲突点 Y。没有冲突时刻时为 None。
    brake_avoidable: bool | None = None  # 无单位。以当前速度和配置的最大减速度能否在冲突距离前停下。未知为 None。
    prediction: str | None = None  # 无单位。运动预测假设。本轮为 constant_velocity_no_steer。未知为 None。

    def __post_init__(self) -> None:
        missing = self.ttc_s is None or self.pred_min_distance_m is None
        if missing and not self.invalid_reason:
            raise ValueError("ttc 或预测最小间距为 None 时必须填写 invalid_reason")
        if not missing and self.invalid_reason is not None:
            raise ValueError("ttc 与预测最小间距都有效时 invalid_reason 必须为 None")


@dataclass(frozen=True)
class ControlRequest:
    """策略请求。这不是车辆本帧的实际执行量。"""

    throttle: float  # 无量纲，范围 0~1。油门请求，无坐标系。
    brake: float  # 无量纲，范围 0~1。制动请求，无坐标系。
    steer: float  # 无量纲，范围 -1~1。转向请求，向右为正，无长度单位。
    desired_speed_mps: float | None  # 单位 m/s。车体纵向期望速度，向前为正。未使用速度请求时为 None。
    target_decel_mps2: float | None = None  # 单位 m/s^2。策略请求的减速度，制动为正。还没经过斜率限制。旧日志没有该字段时为 None。


@dataclass(frozen=True)
class ActualControl:
    """本帧实际下发给执行器的量，与 ControlRequest 分开记录。"""

    throttle: float  # 无量纲，范围 0~1。实际油门，无坐标系。
    brake: float  # 无量纲，范围 0~1。实际制动，无坐标系。
    steer: float  # 无量纲，范围 -1~1。实际转向，向右为正。
    desired_speed_mps: float | None  # 单位 m/s。实际下发的纵向速度命令，向前为正。不是测量车速；没有速度命令时为 None。
    applied_decel_mps2: float | None = None  # 单位 m/s^2。限幅并限斜率之后的减速度，制动为正。不是车速差分得到的实测值。旧日志没有该字段时为 None。


@dataclass(frozen=True)
class FrameRecord:
    """一个仿真步的完整记录。控制请求和实际执行量是两个字段。"""

    run_id: str  # 无单位。一次运行的编号，无坐标系。
    scene_id: str  # 无单位。场景编号，无坐标系。
    strategy_name: str  # 无单位。策略名，无坐标系。
    frame: int  # 无单位。帧号，从 0 递增，无坐标系。
    sim_time_s: float  # 单位 s。仿真时间，同步模式下为帧号乘固定时步。无坐标系。
    source_tag: str  # 无单位。数据来源标签，如 gt 或 perception，无坐标系。
    ego: EgoState  # 自车状态。单位和坐标系见 EgoState。
    threat: ThreatResult  # 威胁结果。单位见 ThreatResult。
    aeb_state: str  # 无单位。AEB 状态：IDLE、WARN、PARTIAL、FULL、EXIT。
    aes_state: str  # 无单位。AES 状态：IDLE、ACTIVE、EXIT。
    trigger_reason: str | None  # 无单位。本帧触发原因。未触发为 None。
    exit_reason: str | None  # 无单位。本帧退出原因。未退出为 None。
    arbitration_reason: str | None  # 无单位。AEB 与 AES 的仲裁原因。未仲裁为 None。
    control_request: ControlRequest  # 策略请求。不是实际执行量。
    actual_control: ActualControl  # 实际执行量：油门、制动、转向、期望速度。
    quality_flags: list[str]  # 无单位。数据质量标志。空列表表示本帧合格。
    aes_direction: str | None = None  # 无单位。选中的避让方向 left 或 right。没有避让时为 None。
    path_error_m: float | None = None  # 单位 m。相对避让轨迹的横向误差，向右为正。没在跟踪时为 None。
    road_departure_m: float | None = None  # 单位 m。车身超出可行驶边界的距离。在界内为 0。未计算为 None。
    side_gap_m: float | None = None  # 单位 m。与侧向目标的最小表面间距；没有侧向目标时是车身到可行驶边界的余量。
    lateral_accel_mps2: float | None = None  # 单位 m/s^2。横向位移的二次导数，向右为正。未计算为 None。


@dataclass(frozen=True)
class EpisodeSummary:
    """一次场景跑完后的汇总。测量不到的量保持 None，不要写成 0。"""

    run_id: str  # 无单位。运行编号，无坐标系。
    scene_id: str  # 无单位。场景编号，无坐标系。
    strategy_name: str  # 无单位。策略名，无坐标系。
    outcome: str  # 无单位。结果标签：avoided、collision 或 incomplete。
    collided: bool  # 无单位。是否发生碰撞。
    min_distance_m: float | None  # 单位 m。全程最小表面间距，非负标量。未知为 None。
    min_ttc_s: float | None  # 单位 s。全程最小有效 TTC。没有有效 TTC 时为 None。
    first_trigger_time_s: float | None  # 单位 s。AEB 或 AES 首次离开 IDLE 的仿真时间。未触发为 None。
    aeb_first_trigger_time_s: float | None  # 单位 s。AEB 首次离开 IDLE 的仿真时间。未触发为 None。
    aes_first_trigger_time_s: float | None  # 单位 s。AES 首次离开 IDLE 的仿真时间。未触发为 None。
    peak_decel_mps2: float | None  # 单位 m/s^2。峰值减速度，取纵向加速度负值的最大者，制动为正。未知为 None。
    speed_drop_mps: float | None  # 单位 m/s。触发后速度下降量，非负。未触发为 None。
    initial_speed_mps: float | None  # 单位 m/s。片段起始水平速率。未知为 None。
    final_speed_mps: float | None  # 单位 m/s。片段结束水平速率。未知为 None。
    duration_s: float  # 单位 s。末帧仿真时间减去首帧仿真时间。
    frame_count: int  # 无单位。写入的帧数。
    invalid_frame_count: int  # 无单位。威胁量无效的帧数。
    brake_delay_frames: int | None = None  # 无单位。请求减速度到实测减速度跨过 0.5 m/s^2 的帧数差。没有制动时为 None。


def to_plain(value: Any) -> Any:
    """把 dataclass 收成可稳定编码的内置对象。None 保持 None。"""
    if is_dataclass(value) and not isinstance(value, type):
        hints = _hints(type(value))
        plain = {}
        for item in fields(value):
            inner = getattr(value, item.name)
            if inner is not None and _is_float_hint(hints[item.name]):
                if isinstance(inner, bool) or not isinstance(inner, (int, float)):
                    raise TypeError(f"{type(value).__name__}.{item.name} 需要数值")
                inner = float(inner)
            plain[item.name] = to_plain(inner)
        return plain
    if isinstance(value, dict):
        return {str(key): to_plain(inner) for key, inner in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_plain(inner) for inner in value]
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    raise TypeError(f"无法序列化 {type(value).__name__}")


def dump_line(value: Any) -> str:
    """一行 JSON，末尾带换行。字段顺序跟 dataclass 定义一致。"""
    return json.dumps(to_plain(value), ensure_ascii=False, allow_nan=False) + "\n"


def dump_pretty(value: Any) -> str:
    """给人打开看的 JSON。null 会写出来，不用省略。"""
    return json.dumps(to_plain(value), ensure_ascii=False, allow_nan=False, indent=2) + "\n"


def load_dataclass(cls, data: dict):
    """从字典还原 dataclass。JSON null 回到 None，未知字段直接报错。"""
    if not isinstance(data, dict):
        raise TypeError(f"{cls.__name__} 需要对象，收到 {type(data).__name__}")
    hints = get_type_hints(cls)
    known = [item.name for item in fields(cls)]
    defaults = _field_defaults(cls)
    extra = sorted(set(data) - set(known))
    missing = sorted(name for name in known if name not in data and name not in defaults)
    if extra:
        raise ValueError(f"{cls.__name__} 含未知字段: {extra}")
    if missing:
        raise ValueError(f"{cls.__name__} 缺少字段: {missing}")
    kwargs = {}
    for name in known:
        if name in data:
            kwargs[name] = _coerce(hints[name], data[name])
        else:
            kwargs[name] = defaults[name]
    return cls(**kwargs)


def parse_line(cls, line: str):
    """解析一行 JSON。要求使用 dump_line 写出的换行。"""
    if line.endswith("\r\n"):
        payload = line[:-2]
    elif line.endswith("\n"):
        payload = line[:-1]
    else:
        payload = line
    return load_dataclass(cls, json.loads(payload))


_HINT_CACHE: dict[type, dict] = {}


def _field_defaults(cls: type) -> dict:
    """只给带默认值的字段补旧日志。没有默认值的字段仍然必须出现。"""
    mapping = {}
    for item in fields(cls):
        if item.default is not MISSING:
            mapping[item.name] = item.default
        elif item.default_factory is not MISSING:
            mapping[item.name] = item.default_factory()
    return mapping


def _hints(cls: type) -> dict:
    cached = _HINT_CACHE.get(cls)
    if cached is None:
        cached = get_type_hints(cls)
        _HINT_CACHE[cls] = cached
    return cached


def _is_float_hint(hint) -> bool:
    if hint is float:
        return True
    origin = get_origin(hint)
    if origin in (Union, types.UnionType):
        args = [arg for arg in get_args(hint) if arg is not type(None)]
        return len(args) == 1 and args[0] is float
    return False


def _coerce(hint, value):
    if value is None:
        return None
    origin = get_origin(hint)
    if origin in (Union, types.UnionType):
        args = [arg for arg in get_args(hint) if arg is not type(None)]
        if len(args) != 1:
            raise TypeError(f"不支持的联合类型 {hint}")
        return _coerce(args[0], value)
    if origin is list:
        inner = get_args(hint)[0]
        if not isinstance(value, list):
            raise TypeError(f"期望 list，收到 {type(value).__name__}")
        return [_coerce(inner, item) for item in value]
    if origin is dict:
        key_hint, value_hint = get_args(hint)
        if not isinstance(value, dict):
            raise TypeError(f"期望 dict，收到 {type(value).__name__}")
        return {
            _coerce(key_hint, key): _coerce(value_hint, item)
            for key, item in value.items()
        }
    if hint is Any:
        return value
    if isinstance(hint, type) and is_dataclass(hint):
        return load_dataclass(hint, value)
    if hint is bool:
        if not isinstance(value, bool):
            raise TypeError(f"期望 bool，收到 {type(value).__name__}")
        return value
    if hint is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"期望 int，收到 {type(value).__name__}")
        return value
    if hint is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError(f"期望 float，收到 {type(value).__name__}")
        return float(value)
    if hint is str:
        if not isinstance(value, str):
            raise TypeError(f"期望 str，收到 {type(value).__name__}")
        return value
    raise TypeError(f"不支持的字段类型 {hint}")
