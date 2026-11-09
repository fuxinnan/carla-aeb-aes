"""闭环跑一个场景。策略可以是无干预、固定 TTC 或状态机 AEB。"""

from __future__ import annotations

import argparse
import json
import math
import random
import signal
import sys
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if not any(Path(entry).resolve() == _PROJECT_ROOT for entry in sys.path if entry):
    sys.path.insert(0, str(_PROJECT_ROOT))

from aeb import build_policy, load_aeb_config
from control.actuator import BrakeActuator
from experiment import build_manifest
from run_logger import RunLogger
from scenarios.closed_loop import make_summary, simulate_episode
from scenarios.geometry import lane_boundaries, surface_gap
from scenarios.placement import find_straight_road
from scenarios.report import assess_repeats, format_repeat_report
from scenarios.spawn import lead_vehicle_brake, pedestrian_crossing
from scenarios.spec import ScenarioSpec, load_spec
from schemas import FrameRecord
from sim.carla_stage import follow_spectator
from sim.debug_draw import draw_scenario_debug
from sim.carla_session import CarlaSession, server_synchronous_mode
from sim.empty_loop import check_frame_clock
from viz.replay_bev import export_bev_replay
from viz.scenario_preview import save_scenario_preview
from viz.stability import save_stability_plot


SPAWNERS = {
    "pedestrian_crossing": pedestrian_crossing,
    "lead_vehicle_brake": lead_vehicle_brake,
}


def run_repeats(
    config_path: str | Path,
    *,
    repeats: int = 20,
    run_root: str | Path | None = None,
    debug_draw: bool = False,
    strategy: str = "no_intervention",
    aeb_config: str | Path | None = None,
    pace_s: float = 0.0,
) -> dict:
    """每次重复都清掉上一轮 actor 再生成。种子来自场景，不随策略改变。"""
    if repeats <= 0:
        raise ValueError("repeats 必须大于 0")
    spec = load_spec(config_path)
    config = load_aeb_config(aeb_config)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    group = Path(run_root) if run_root is not None else _PROJECT_ROOT / "runs" / "round5" / f"{spec.scene_id}_{strategy}_{stamp}"
    group.mkdir(parents=True, exist_ok=True)
    results = []
    preview_road = None
    with CarlaSession(
        host=spec.host,
        port=spec.port,
        map_name=spec.map_name,
        fixed_delta_seconds=spec.fixed_delta_seconds,
        render=True,
        tm_port=spec.tm_port,
        traffic_manager_seed=spec.seed,
        spawn_ego=False,
        debug_draw=debug_draw,
    ) as session:
        print(f"map ready server={session.server_version}", flush=True)
        for index in range(repeats):
            if index:
                print(f"clear actors repeat={index:02d}", flush=True)
                session.reload_episode()
            episode = run_episode(
                spec,
                group / f"rep_{index:02d}",
                session,
                strategy=strategy,
                aeb_config=config,
                pace_s=pace_s,
            )
            episode["index"] = index
            results.append(episode)
            if preview_road is None:
                preview_road = episode["road"]
            print(
                f"repeat={index:02d} strategy={strategy} trigger={episode['trigger_frame']} "
                f"collided={int(episode['collided'])} "
                f"relative_speed_mps={_fmt(episode['collision_relative_speed_mps'])} "
                f"brake_delay_frames={episode['brake_delay_frames']} "
                f"run_dir={episode['run_dir']}"
            )
    report = assess_repeats(results)
    _write_group_outputs(group, spec, results, report, preview_road)
    if server_synchronous_mode(spec.host, spec.port):
        raise RuntimeError("退出后服务端仍停在同步模式")
    report["group_dir"] = group
    report["scene_id"] = spec.scene_id
    report["hard_negative"] = spec.hard_negative
    report["pose_ok"] = all(item["pose_ok"] for item in results)
    report["timeline"] = results[0]["timeline"]
    report["snapshot"] = results[0]["snapshot"]
    report["diagnostic"] = results[0]["diagnostic"]
    report["mermaid"] = results[0]["mermaid"]
    report["state_png"] = results[0]["state_png"]
    report["strategy"] = strategy
    report["brake_delay_frames"] = [item["brake_delay_frames"] for item in results]
    return report


def run_episode(
    spec: ScenarioSpec,
    run_dir: Path,
    session: CarlaSession | None = None,
    *,
    debug_draw: bool = False,
    strategy: str = "no_intervention",
    aeb_config=None,
    scenario_split: str | None = None,
    pace_s: float = 0.0,
) -> dict:
    """session 为空时自己开关一次世界。重复实验传入同一个会话，并在外部清掉 actor。"""
    _seed(spec.seed)
    if session is None:
        print(f"load map={spec.map_name} run_dir={run_dir}", flush=True)
        with CarlaSession(
            host=spec.host,
            port=spec.port,
            map_name=spec.map_name,
            fixed_delta_seconds=spec.fixed_delta_seconds,
            render=True,
            tm_port=spec.tm_port,
            traffic_manager_seed=spec.seed,
            spawn_ego=False,
            debug_draw=debug_draw,
        ) as owned:
            print(f"map ready server={owned.server_version}", flush=True)
            return _execute(owned, spec, run_dir, strategy, aeb_config, scenario_split, pace_s)
    print(f"episode run_dir={run_dir}", flush=True)
    return _execute(session, spec, run_dir, strategy, aeb_config, scenario_split, pace_s)


def _execute(session: CarlaSession, spec: ScenarioSpec, run_dir: Path, strategy: str, aeb_config, scenario_split: str | None = None, pace_s: float = 0.0) -> dict:
    road, spawn_index = find_straight_road(session.world, spec.straight_length_m, spec.sample_step_m)
    print(f"road spawn_index={spawn_index} samples={len(road)} strategy={strategy}", flush=True)
    config = aeb_config if aeb_config is not None and not isinstance(aeb_config, (str, Path)) else load_aeb_config(aeb_config)
    policy = build_policy(strategy, config, spec.ego_speed_mps)
    actuator = BrakeActuator(config.actuator, spec.fixed_delta_seconds)
    bound = replace(spec, runtime=(session, road))
    actors = SPAWNERS[spec.template](bound)
    print(f"spawned ego={actors.ego.id} target={actors.target.id}", flush=True)
    fields = spec.key_fields()
    left, right = lane_boundaries(road)
    fields["road"]["lane_half_width_m"] = road[0].lane_half_width_m
    fields["road"]["spawn_index"] = spawn_index
    fields["road"]["boundaries"] = {
        "left": [{"x_m": x, "y_m": y} for x, y in left],
        "right": [{"x_m": x, "y_m": y} for x, y in right],
    }
    manifest = build_manifest(
        run_id=run_dir.name,
        scene_id=spec.scene_id,
        strategy_name=policy.name,
        strategy_version=policy.version,
        strategy_config_path=config.path,
        carla_version=session.server_version,
        map_name=spec.map_name,
        fixed_delta_seconds=spec.fixed_delta_seconds,
        synchronous_mode=True,
        physics_substep=session.physics_substep(),
        python_seed=spec.seed,
        numpy_seed=spec.seed,
        traffic_manager_seed=spec.seed,
        weather=session.weather_dict(),
    )
    manifest["scenario"] = fields
    if scenario_split is not None:
        if scenario_split not in {"tuning", "test"}:
            raise ValueError("场景集只能是 tuning 或 test")
        manifest["evaluation"] = {
            "split": scenario_split,
            "used_for_tuning": scenario_split == "tuning",
            "seed_source": "scenario_yaml",
        }
    with RunLogger(run_dir, manifest) as logger:
        logger.log_event("scenario_spec", 0.0, fields)
        driven = _drive(session, spec, actors, road, policy, actuator, logger, run_dir.name, pace_s)
        logger.update_manifest({"scenario_result": driven["public"]})
        logger.write_summary(driven["summary"])
    records = driven["records"]
    if records and not driven["interrupted"]:
        check_frame_clock(records, spec.fixed_delta_seconds)
    figures = _save_figures(run_dir)
    return {
        "run_dir": run_dir,
        "road": road,
        "trigger_frame": driven["trigger_frame"],
        "collision_frame": driven["collision_frame"],
        "collided": driven["collided"],
        "collision_relative_speed_mps": driven["collision_relative_speed_mps"],
        "brake_delay_frames": driven["summary"].brake_delay_frames,
        "target_spawn": driven["target_spawn"],
        "ego_spawn": driven["ego_spawn"],
        "ego_xy": driven["ego_xy"],
        "target_xy": driven["target_xy"],
        "trigger_xy": driven["trigger_xy"],
        "collision_xy": driven["collision_xy"],
        "end_reason": driven["end_reason"],
        "pose_ok": driven["pose_ok"],
        "timeline": figures["timeline"],
        "snapshot": figures["snapshot"],
        "diagnostic": figures["diagnostic"],
        "mermaid": figures["mermaid"],
        "state_png": figures["state_png"],
    }


def _drive(session, spec: ScenarioSpec, actors, road, policy, actuator, logger: RunLogger, run_id: str, pace_s: float = 0.0) -> dict:
    episode = simulate_episode(spec, road, policy, actuator, run_id)
    records: list[FrameRecord] = []
    gaps: list[float] = []
    mismatch_frames = 0
    interrupted = False
    ego_spawn = None
    target_spawn = None
    ego_xy = []
    target_xy = []
    trigger_xy = None
    collision_xy = None
    import time

    life_s = max(spec.fixed_delta_seconds, pace_s)
    try:
        for step in episode.steps:
            started = time.perf_counter()
            _apply(actors.ego, step.ego)
            _apply(actors.target, step.target)
            follow_spectator(
                session.world,
                step.ego.x_m,
                step.ego.y_m,
                step.ego.z_m,
                step.ego.yaw_rad,
                back_m=18.0,
                height_m=10.0,
                pitch_deg=-22.0,
            )
            session.tick()
            ego_pose = _read_pose(actors.ego)
            target_pose = _read_pose(actors.target)
            if ego_spawn is None:
                ego_spawn = ego_pose
                target_spawn = target_pose
            flags = list(step.record.quality_flags)
            if _pose_error(ego_pose, step.ego) > 0.05 or _pose_error(target_pose, step.target) > 0.05:
                flags.append("pose_mismatch")
                mismatch_frames += 1
            gap = surface_gap(
                ego_pose["x_m"],
                ego_pose["y_m"],
                ego_pose["yaw_rad"],
                spec.ego_length_m,
                spec.ego_width_m,
                target_pose["x_m"],
                target_pose["y_m"],
                target_pose["yaw_rad"],
                spec.target_length_m,
                spec.target_width_m,
            )
            record = replace(
                step.record,
                ego=replace(
                    step.record.ego,
                    x_m=ego_pose["x_m"],
                    y_m=ego_pose["y_m"],
                    z_m=ego_pose["z_m"],
                    yaw_rad=ego_pose["yaw_rad"],
                ),
                quality_flags=flags,
            )
            if session.debug_draw:
                notice = None
                if step.transition is not None:
                    notice = f"AEB {step.transition.current} {step.transition.reason}"
                draw_scenario_debug(
                    session.world,
                    actors.ego,
                    actors.target,
                    ego_length_m=spec.ego_length_m,
                    ego_width_m=spec.ego_width_m,
                    ego_height_m=spec.ego_height_m,
                    target_length_m=spec.target_length_m,
                    target_width_m=spec.target_width_m,
                    target_height_m=spec.target_height_m,
                    threat=record.threat,
                    life_time_s=life_s,
                    aeb_state=record.aeb_state,
                    notice=notice,
                )
            logger.log_frame(record)
            for track in step.tracks:
                logger.log_target(step.frame, step.sim_time_s, track)
            if step.transition is not None:
                logger.log_event(
                    "aeb_state",
                    step.sim_time_s,
                    {
                        "frame": step.frame,
                        "from": step.transition.previous,
                        "to": step.transition.current,
                        "reason": step.transition.reason,
                        "ttc_s": record.threat.ttc_s,
                    },
                )
            records.append(record)
            gaps.append(gap)
            ego_xy.append((step.ego.x_m, step.ego.y_m))
            target_xy.append((step.target.x_m, step.target.y_m))
            if step.trigger_edge:
                trigger_xy = (step.ego.x_m, step.ego.y_m)
                logger.log_event(
                    "target_trigger",
                    step.sim_time_s,
                    {"frame": step.frame, "distance_m": step.distance_m},
                )
            if step.overlapping:
                collision_xy = (
                    (ego_pose["x_m"] + target_pose["x_m"]) * 0.5,
                    (ego_pose["y_m"] + target_pose["y_m"]) * 0.5,
                )
                logger.log_event(
                    "collision",
                    step.sim_time_s,
                    {
                        "frame": step.frame,
                        "detector": "oriented_box",
                        "relative_speed_mps": episode.collision_relative_speed_mps,
                        "surface_gap_m": gap,
                        "ego_x_m": ego_pose["x_m"],
                        "ego_y_m": ego_pose["y_m"],
                        "target_x_m": target_pose["x_m"],
                        "target_y_m": target_pose["y_m"],
                    },
                )
            if pace_s > 0.0:
                remaining = pace_s - (time.perf_counter() - started)
                if remaining > 0.0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        interrupted = True
    if not records:
        raise RuntimeError("场景没有写下任何一帧")
    end_reason = "incomplete" if interrupted else episode.end_reason
    collided = episode.collided and not interrupted
    if interrupted:
        outcome = "incomplete"
    elif collided:
        outcome = "collision"
    else:
        outcome = "avoided"
    summary = make_summary(spec, run_id, policy.name, records, gaps, collided, outcome)
    public = {
        "trigger_frame": None if interrupted else episode.trigger_frame,
        "collision_frame": None if interrupted else episode.collision_frame,
        "collided": collided,
        "outcome": outcome,
        "end_reason": end_reason,
        "collision_relative_speed_mps": None if interrupted else episode.collision_relative_speed_mps,
        "brake_delay_frames": summary.brake_delay_frames,
        "hard_negative": spec.hard_negative,
        "ego_spawn": ego_spawn,
        "target_spawn": target_spawn,
    }
    logger.log_event("scenario_end", records[-1].sim_time_s, public)
    return {
        "records": records,
        "summary": summary,
        "public": public,
        "trigger_frame": public["trigger_frame"],
        "collision_frame": public["collision_frame"],
        "collided": collided,
        "collision_relative_speed_mps": public["collision_relative_speed_mps"],
        "target_spawn": target_spawn,
        "ego_spawn": ego_spawn,
        "ego_xy": ego_xy,
        "target_xy": target_xy,
        "trigger_xy": trigger_xy,
        "collision_xy": collision_xy,
        "end_reason": end_reason,
        "interrupted": interrupted,
        "pose_ok": mismatch_frames == 0,
    }


def _apply(actor, sample) -> None:
    import carla

    actor.set_transform(
        carla.Transform(
            carla.Location(x=float(sample.x_m), y=float(sample.y_m), z=float(sample.z_m)),
            carla.Rotation(pitch=0.0, yaw=math.degrees(float(sample.yaw_rad)), roll=0.0),
        )
    )
    actor.set_target_velocity(carla.Vector3D(float(sample.v_x_mps), float(sample.v_y_mps), 0.0))


def _read_pose(actor) -> dict:
    transform = actor.get_transform()
    return {
        "x_m": float(transform.location.x),
        "y_m": float(transform.location.y),
        "z_m": float(transform.location.z),
        "yaw_rad": math.radians(float(transform.rotation.yaw)),
    }


def _pose_error(measured: dict, sample) -> float:
    return math.hypot(measured["x_m"] - sample.x_m, measured["y_m"] - sample.y_m, measured["z_m"] - sample.z_m)


def _save_figures(run_dir: Path) -> dict:
    from viz.aeb_diag import plot_aeb_diagnostic, write_state_diagram
    from viz.threat import plot_threat_snapshot, plot_threat_timeline

    mermaid, state_png = write_state_diagram(run_dir)
    return {
        "timeline": plot_threat_timeline(run_dir),
        "snapshot": plot_threat_snapshot(run_dir),
        "diagnostic": plot_aeb_diagnostic(run_dir),
        "mermaid": mermaid,
        "state_png": state_png,
    }


def _write_group_outputs(group, spec, results, report, road) -> None:
    left, right = lane_boundaries(road)
    first = results[0]
    preview = save_scenario_preview(
        group / "scenario_preview.png",
        spec_fields=spec.key_fields(),
        ego_xy=first["ego_xy"],
        target_xy=first["target_xy"],
        left_xy=left,
        right_xy=right,
        trigger_xy=first["trigger_xy"],
        collision_xy=first["collision_xy"],
    )
    stability = save_stability_plot(
        group / "stability.png",
        [{"index": item["index"], "trigger_frame": item["trigger_frame"], "collision_relative_speed_mps": item["collision_relative_speed_mps"], "collided": item["collided"]} for item in results],
        title=spec.scene_id,
    )
    replay = export_bev_replay(results[0]["run_dir"])
    payload = {
        "scene_id": spec.scene_id,
        "config": str(spec.config_path),
        "stable": report["stable"],
        "target_spawn_deviation_m": report["target_spawn_deviation_m"],
        "trigger_frames": report["trigger_frames"],
        "collided": report["collided"],
        "collision_relative_speed_mps": report["collision_relative_speed_mps"],
        "speed_range_mps": report["speed_range_mps"],
        "preview": str(preview),
        "stability": str(stability),
        "replay": str(replay),
        "runs": [
            {
                "index": item["index"],
                "run_dir": str(item["run_dir"]),
                "trigger_frame": item["trigger_frame"],
                "collided": item["collided"],
                "collision_relative_speed_mps": item["collision_relative_speed_mps"],
                "brake_delay_frames": item["brake_delay_frames"],
                "target_spawn": item["target_spawn"],
            }
            for item in results
        ],
    }
    (group / "repeats.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    report["preview"] = preview
    report["stability"] = stability
    report["replay"] = replay


def _seed(seed: int) -> None:
    random.seed(seed)
    try:
        import numpy as np
    except Exception:
        return
    np.random.seed(seed)


def _fmt(value) -> str:
    if value is None:
        return "null"
    return f"{float(value):.6f}"


def _install_break_handler() -> None:
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, _raise_keyboard_interrupt)


def _raise_keyboard_interrupt(_signum, _frame) -> None:
    raise KeyboardInterrupt


def main(argv: list[str] | None = None) -> int:
    _install_break_handler()
    parser = argparse.ArgumentParser(description="用选定策略闭环跑一个场景实例。")
    parser.add_argument("--config", required=True, help="参数实例 YAML")
    parser.add_argument("--strategy", default="no_intervention", choices=["no_intervention", "fixed_ttc", "state_machine", "aeb_aes"])
    parser.add_argument("--aeb-config", default=None, help="AEB 与执行层 YAML，默认 configs/aeb/round5.yaml")
    parser.add_argument("--repeats", type=int, default=20, help="同一 YAML、同一种子的重复次数")
    parser.add_argument("--run-root", default=None, help="结果目录，默认写到 runs/round5 下按时间新建")
    parser.add_argument("--debug-draw", action=argparse.BooleanOptionalAction, default=True, help="自车框按 AEB 状态着色，切换帧写 ASCII 原因")
    parser.add_argument("--pace", type=float, default=0.25, help="每一帧在屏幕上停留的秒数。0 表示按机器最快速度播")
    args = parser.parse_args(argv)
    if args.pace < 0.0:
        raise SystemExit("--pace 不能为负")
    spec = load_spec(args.config)
    print(f"view pace_s={args.pace:.2f} ego_speed_mps={spec.ego_speed_mps:.1f}", flush=True)
    report = run_repeats(
        args.config,
        repeats=args.repeats,
        run_root=args.run_root,
        debug_draw=args.debug_draw,
        strategy=args.strategy,
        aeb_config=args.aeb_config,
        pace_s=args.pace,
    )
    print(format_repeat_report(spec.scene_id, report))
    print(f"preview={report['preview']}")
    print(f"stability={report['stability']}")
    print(f"replay={report['replay']}")
    print(f"timeline={report['timeline']}")
    print(f"snapshot={report['snapshot']}")
    print(f"diagnostic={report['diagnostic']}")
    print(f"mermaid={report['mermaid']}")
    print(f"state_png={report['state_png']}")
    print(f"brake_delay_frames={report['brake_delay_frames']}")
    print(f"group_dir={report['group_dir']}")
    if args.strategy == "no_intervention":
        collided = report["collided"]
        if spec.hard_negative:
            outcome_ok = all(not item for item in collided)
        else:
            outcome_ok = all(collided)
        if not (report["stable"] and outcome_ok):
            print("重复结果不稳定，或无干预结果和硬负例标记不一致。先看 speed_range 和 trigger_frames。", file=sys.stderr)
            return 2
    elif not report["stable"]:
        print("重复结果不稳定。先看 speed_range 和 trigger_frames。", file=sys.stderr)
        return 2
    if not report["pose_ok"]:
        print("有单帧位姿回读偏差。触发帧、是否碰撞和相对速度仍然一致。", file=sys.stderr)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
