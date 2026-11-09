"""只在调参集上选阈值。测试集要等配置写好之后才读取。"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import yaml

from aeb import build_policy, load_aeb_config
from aeb.config import AebConfig, FixedTtcConfig, StateMachineConfig
from control.actuator import BrakeActuator
from metrics.episode import evaluate_run
from round9.catalog import DT_S, TEMPLATES, GridScene, load_split, write_grid
from round9.simulate import simulate_scene
from schemas import dump_line, dump_pretty


OBJECTIVE = "collision_rate + 0.5 * hard_negative_false_brake_rate"
FIXED_THRESHOLDS = (1.0, 1.4, 1.8, 2.2)
STATE_CONFIRMS = (1, 3)
STATE_PARTIALS = (1.4, 1.8)
STRATEGIES = ("no_intervention", "fixed_ttc", "state_machine", "aeb_aes")
HYSTERESIS_GAP_S = 0.05


@dataclass(frozen=True)
class Choice:
    kind: str
    label: str
    score: float
    false_rate: float
    switches: float
    config: AebConfig


class Journal:
    """调参阶段打开过的场景文件。测试目录单独记，不写进调参记录。"""

    def __init__(self) -> None:
        self.opened: list[str] = []
        self.test_dir_opened = False

    def load_tuning(self, root: str | Path) -> list[GridScene]:
        scenes = load_split(Path(root) / "tuning", "tuning")
        for scene in scenes:
            path = str(scene.path)
            self.opened.append(path)
            if Path(path).parent.name == "test":
                self.test_dir_opened = True
        return scenes

    def load_test(self, root: str | Path) -> list[GridScene]:
        self.test_dir_opened = True
        return load_split(Path(root) / "test", "test")

    def snapshot(self) -> dict:
        if self.test_dir_opened:
            raise RuntimeError("调参记录里不能出现测试集")
        return {
            "objective": OBJECTIVE,
            "lower_is_better": True,
            "test_dir_opened": False,
            "opened_paths": list(self.opened),
            "opened_count": len(self.opened),
        }


def write_aeb_yaml(path: Path, config: AebConfig) -> AebConfig:
    """写成完整配置再读回来。加载器会拒绝迟滞写反。"""
    machine = config.state_machine
    payload = {
        "version": config.version,
        "fixed_ttc": {
            "ttc_brake_s": round(config.fixed_ttc.ttc_brake_s, 3),
            "target_decel_mps2": round(config.fixed_ttc.target_decel_mps2, 3),
        },
        "state_machine": {
            "confirm_frames": machine.confirm_frames,
            "min_hold_frames": machine.min_hold_frames,
            "lost_hold_frames": machine.lost_hold_frames,
            "clear_frames": machine.clear_frames,
            "stop_speed_mps": round(machine.stop_speed_mps, 3),
            "ttc_warn_on_s": round(machine.ttc_warn_on_s, 3),
            "ttc_warn_off_s": round(machine.ttc_warn_off_s, 3),
            "ttc_partial_on_s": round(machine.ttc_partial_on_s, 3),
            "ttc_partial_off_s": round(machine.ttc_partial_off_s, 3),
            "ttc_full_on_s": round(machine.ttc_full_on_s, 3),
            "ttc_full_off_s": round(machine.ttc_full_off_s, 3),
            "partial_decel_mps2": machine.partial_decel_mps2,
            "full_decel_mps2": machine.full_decel_mps2,
        },
        "actuator": {
            "brake_per_decel": config.actuator.brake_per_decel,
            "max_brake": config.actuator.max_brake,
            "max_decel_mps2": config.actuator.max_decel_mps2,
            "slew_decel_mps3": config.actuator.slew_decel_mps3,
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return load_aeb_config(path)


def candidate_configs(base: AebConfig) -> list[tuple[str, str, AebConfig]]:
    """固定阈值和状态机分开搜。状态机只动确认帧数和部分制动进入阈值，其余档位按固定间隙排好。"""
    found = []
    for threshold in FIXED_THRESHOLDS:
        token = str(threshold).replace(".", "_")
        config = AebConfig(
            version="round9-search",
            fixed_ttc=FixedTtcConfig(threshold, base.fixed_ttc.target_decel_mps2),
            state_machine=base.state_machine,
            actuator=base.actuator,
            path=base.path,
        )
        found.append(("fixed_ttc", f"fixed_{token}", config))
    for confirm in STATE_CONFIRMS:
        for partial in STATE_PARTIALS:
            token = f"state_c{confirm}_p{str(partial).replace('.', '_')}"
            config = AebConfig(
                version="round9-search",
                fixed_ttc=base.fixed_ttc,
                state_machine=_chain(base.state_machine, confirm, partial, tight=False),
                actuator=base.actuator,
                path=base.path,
            )
            found.append(("state_machine", token, config))
    return found


def ablation_configs(chosen: AebConfig) -> list[tuple[str, str, AebConfig]]:
    """三次消融都从选定的状态机改一个配置项。迟滞不能写成相等，加载器要求严格大于。"""
    machine = chosen.state_machine
    tight = _replace_machine(
        machine,
        ttc_warn_off_s=machine.ttc_warn_on_s + HYSTERESIS_GAP_S,
        ttc_partial_off_s=machine.ttc_partial_on_s + HYSTERESIS_GAP_S,
        ttc_full_off_s=machine.ttc_full_on_s + HYSTERESIS_GAP_S,
    )
    no_confirm = _replace_machine(machine, confirm_frames=1)
    no_grade = _replace_machine(machine, partial_decel_mps2=machine.full_decel_mps2)
    rows = [
        ("full", "完整状态机", chosen),
        ("no_hysteresis", "去掉迟滞", _swap(chosen, tight, "round9-ablation")),
        ("no_confirm", "去掉风险确认", _swap(chosen, no_confirm, "round9-ablation")),
        ("no_grading", "去掉分级制动", _swap(chosen, no_grade, "round9-ablation")),
    ]
    return rows


def run_episode(scene: GridScene, strategy: str, config: AebConfig, directory: Path, *, occlusion: bool, stage=None) -> dict:
    """stage 是打开的 CARLA 世界。实验入口必须传入，这样每一帧都会 tick。"""
    policy = build_policy(strategy, config, scene.ego_speed_mps)
    actuator = BrakeActuator(config.actuator, DT_S)
    result = simulate_scene(scene, policy, actuator, occlusion=occlusion)
    if stage is not None:
        print(f"carla {scene.scene_id} {strategy} frames={len(result.samples)}", flush=True)
        stage.play(result.samples, result.records, dt_s=DT_S)
    _write_logs(directory, scene, strategy, result, None if stage is None else stage.playback_meta())
    row = evaluate_run(directory)
    row["template"] = scene.template
    row["ego_speed_mps"] = scene.ego_speed_mps
    row["distance_m"] = scene.distance_m
    row["split"] = scene.split
    return row


def score_rows(rows: list[dict]) -> tuple[float, float, float]:
    if not rows:
        raise ValueError("没有可评分的运行")
    collision = sum(1.0 for row in rows if row["collided"]) / len(rows)
    hard = [row for row in rows if row["hard_negative"]]
    false_rate = 0.0 if not hard else sum(1.0 for row in hard if row["false_brake"]) / len(hard)
    switches = sum(row["state_switch_count"] for row in rows) / len(rows)
    return collision + 0.5 * false_rate, false_rate, switches


def choose(kind: str, ranked: list[Choice]) -> Choice:
    """分数低优先。同分再看误制动、状态切换，最后靠近第 5 轮占位值。"""
    pool = [item for item in ranked if item.kind == kind]
    if not pool:
        raise ValueError(f"没有 {kind} 的候选")
    return min(pool, key=lambda item: (item.score, item.false_rate, item.switches, _default_distance(item)))


def _chain(base: StateMachineConfig, confirm: int, partial_on: float, *, tight: bool) -> StateMachineConfig:
    full_on = partial_on - 0.7
    if tight:
        full_off = full_on + HYSTERESIS_GAP_S
        partial_off = partial_on + HYSTERESIS_GAP_S
        warn_on = partial_off + 0.4
        warn_off = warn_on + HYSTERESIS_GAP_S
    else:
        full_off = full_on + 0.3
        partial_off = partial_on + 0.4
        warn_on = partial_off + 0.4
        warn_off = warn_on + 0.4
    return _replace_machine(
        base,
        confirm_frames=confirm,
        ttc_warn_on_s=warn_on,
        ttc_warn_off_s=warn_off,
        ttc_partial_on_s=partial_on,
        ttc_partial_off_s=partial_off,
        ttc_full_on_s=full_on,
        ttc_full_off_s=full_off,
    )


def _replace_machine(base: StateMachineConfig, **updates) -> StateMachineConfig:
    data = {field: getattr(base, field) for field in base.__dataclass_fields__}
    data.update(updates)
    return StateMachineConfig(**data)


def _swap(config: AebConfig, machine: StateMachineConfig, version: str) -> AebConfig:
    return AebConfig(
        version=version,
        fixed_ttc=config.fixed_ttc,
        state_machine=machine,
        actuator=config.actuator,
        path=config.path,
    )


def _default_distance(choice: Choice) -> float:
    if choice.kind == "fixed_ttc":
        return abs(choice.config.fixed_ttc.ttc_brake_s - 1.4)
    machine = choice.config.state_machine
    return abs(machine.confirm_frames - 3) + abs(machine.ttc_partial_on_s - 1.6)


def _write_logs(directory: Path, scene: GridScene, strategy: str, result, carla_meta: dict | None = None) -> None:
    directory.mkdir(parents=True, exist_ok=False)
    manifest = {
        "scene_id": scene.scene_id,
        "strategy_name": strategy,
        "split": scene.split,
        "seed": scene.seed,
        "evaluation": {"split": scene.split, "used_for_tuning": scene.split == "tuning"},
    }
    if carla_meta is not None:
        manifest["carla"] = carla_meta
    (directory / "manifest.json").write_text(dump_pretty(manifest), encoding="utf-8")
    (directory / "targets.jsonl").write_text("", encoding="utf-8")
    with (directory / "frames.jsonl").open("w", encoding="utf-8", newline="\n") as handle:
        for record in result.records:
            handle.write(dump_line(record))
    with (directory / "events.jsonl").open("w", encoding="utf-8", newline="\n") as handle:
        for event in result.events:
            handle.write(dump_line(event))
    (directory / "summary.json").write_text(dump_pretty(result.summary), encoding="utf-8")


def scene_uses_occlusion(scene: GridScene) -> bool:
    return scene.template == "occlusion_reveal"


def tuning_templates() -> tuple[str, ...]:
    return TEMPLATES
