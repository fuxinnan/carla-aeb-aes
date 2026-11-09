"""按场景目录跑三种策略。种子用 YAML 里的，test 集不改阈值。"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime, timezone
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent
if not any(Path(entry).resolve() == _PROJECT_ROOT for entry in sys.path if entry):
    sys.path.insert(0, str(_PROJECT_ROOT))

from aeb import STRATEGY_NAMES, load_aeb_config
from metrics.report import build_report, collect_runs
from scenarios.runner import run_episode
from scenarios.spec import load_spec
from sim.carla_session import CarlaSession, server_synchronous_mode


def run_scene_directory(
    scene_dir: str | Path,
    *,
    repeats: int = 1,
    output: str | Path | None = None,
    aeb_config: str | Path | None = None,
) -> Path:
    """一个目录里的每个 YAML 都用三种策略跑。两次运行之间只清 actor，不重新加载地图。"""
    folder = Path(scene_dir)
    split = folder.name
    if split not in {"tuning", "test"}:
        raise ValueError("场景目录名必须是 tuning 或 test，规则按目录名写入清单")
    if repeats <= 0:
        raise ValueError("repeat 必须大于 0")
    paths = sorted(folder.glob("*.yaml"))
    if not paths:
        raise ValueError(f"{folder} 里没有场景 YAML")
    specs = [load_spec(path) for path in paths]
    hosts = {(spec.host, spec.port, spec.map_name, spec.fixed_delta_seconds) for spec in specs}
    if len(hosts) != 1:
        raise ValueError("同一场景目录里的地图、端口和时步必须相同")
    config = load_aeb_config(aeb_config)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    root = Path(output) if output is not None else _PROJECT_ROOT / "runs" / "round6" / f"{split}_{stamp}"
    root.mkdir(parents=True, exist_ok=True)
    first = specs[0]
    started = False
    with CarlaSession(
        host=first.host,
        port=first.port,
        map_name=first.map_name,
        fixed_delta_seconds=first.fixed_delta_seconds,
        render=True,
        tm_port=first.tm_port,
        traffic_manager_seed=first.seed,
        spawn_ego=False,
        debug_draw=True,
    ) as session:
        print(f"map ready split={split} server={session.server_version}", flush=True)
        for spec in specs:
            for strategy in STRATEGY_NAMES:
                for index in range(repeats):
                    if started:
                        print(f"clear actors scene={spec.scene_id} strategy={strategy} repeat={index:02d}", flush=True)
                        session.reload_episode()
                    started = True
                    run_dir = root / spec.scene_id / strategy / f"rep_{index:02d}"
                    print(f"run scene={spec.scene_id} strategy={strategy} repeat={index:02d}", flush=True)
                    run_episode(
                        spec,
                        run_dir,
                        session,
                        debug_draw=True,
                        strategy=strategy,
                        aeb_config=config,
                        scenario_split=split,
                    )
    if server_synchronous_mode(first.host, first.port):
        raise RuntimeError("退出后服务端仍停在同步模式")
    rows = collect_runs(root)
    note = "调参集。test 集不参与本轮调参。" if split == "tuning" else "测试集。本轮只出对照，没有用这些结果改阈值或指标。"
    report = build_report(rows, root, f"第6轮 {split} 对照", note)
    print(f"report={report}", flush=True)
    print(f"episodes={len(rows)} output={root}", flush=True)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="批量跑一个场景目录，并写出对照表。")
    parser.add_argument("--scenes", required=True, help="tuning 或 test 场景目录")
    parser.add_argument("--repeat", type=int, default=1, help="同一配置的重复次数")
    parser.add_argument("--output", default=None, help="结果目录，默认写到 runs/round6 下")
    parser.add_argument("--aeb-config", default=None, help="AEB YAML，默认 configs/aeb/round5.yaml")
    args = parser.parse_args(argv)
    run_scene_directory(args.scenes, repeats=args.repeat, output=args.output, aeb_config=args.aeb_config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
