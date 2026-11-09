"""只读清单里的场景和配置，重新跑一遍。不回去打开原来的帧日志。"""

from __future__ import annotations

from pathlib import Path

from aeb import build_policy, load_aeb_config
from control.actuator import BrakeActuator
from round10.guard import ROOT
from round9.catalog import DT_S, GridScene
from round9.simulate import simulate_scene


def replay(manifest: dict, stage=None):
    """manifest 里要有模板、速度、距离、策略和配置路径。传入 stage 时，每一帧都在 CARLA 里 tick。"""
    required = {
        "template",
        "instance",
        "seed",
        "split",
        "ego_speed_mps",
        "distance_m",
        "hard_negative",
        "occlusion",
        "strategy_name",
        "aeb_config",
        "scene_id",
    }
    missing = sorted(required - set(manifest))
    if missing:
        raise ValueError(f"回放清单缺少 {missing}")
    config_path = Path(manifest["aeb_config"])
    if not config_path.is_absolute():
        config_path = ROOT / config_path
    scene = GridScene(
        template=str(manifest["template"]),
        instance=str(manifest["instance"]),
        seed=int(manifest["seed"]),
        split=str(manifest["split"]),
        ego_speed_mps=float(manifest["ego_speed_mps"]),
        distance_m=float(manifest["distance_m"]),
        hard_negative=bool(manifest["hard_negative"]),
    )
    config = load_aeb_config(config_path)
    policy = build_policy(manifest["strategy_name"], config, scene.ego_speed_mps)
    result = simulate_scene(scene, policy, BrakeActuator(config.actuator, DT_S), occlusion=bool(manifest["occlusion"]))
    if stage is not None:
        print(f"carla replay {scene.scene_id} {manifest['strategy_name']} frames={len(result.samples)}", flush=True)
        stage.play(result.samples, result.records, dt_s=DT_S)
    return result
