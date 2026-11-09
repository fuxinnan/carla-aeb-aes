"""空控制闭环：读取快照，推进一帧，收集响应，写日志。控制量保持为零。"""

from __future__ import annotations

import argparse
import math
import os
import random
import signal
import sys
from datetime import datetime, timezone
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if not any(Path(entry).resolve() == _PROJECT_ROOT for entry in sys.path if entry):
    sys.path.insert(0, str(_PROJECT_ROOT))

import yaml

from experiment import build_manifest
from run_logger import FRAMES_NAME, RunLogger
from schemas import (
    AEB_IDLE,
    AES_IDLE,
    ActualControl,
    ControlRequest,
    EpisodeSummary,
    FrameRecord,
    ThreatResult,
    parse_line,
)
from sim.carla_session import (
    ROLE_NAME,
    SPAWN_INDEX,
    TM_PORT,
    VEHICLE_BLUEPRINT,
    CarlaSession,
    server_synchronous_mode,
)


ZERO_REQUEST = ControlRequest(throttle=0.0, brake=0.0, steer=0.0, desired_speed_mps=None)
ZERO_APPLIED = ActualControl(throttle=0.0, brake=0.0, steer=0.0, desired_speed_mps=None)
NO_THREAT = ThreatResult(
    ttc_s=None,
    pred_min_distance_m=None,
    invalid_reason="no_threat",
    threat_id=None,
    in_path=None,
    closing_speed_mps=None,
    overlap_ratio=None,
)
CLOCK_TOLERANCE_S = 1e-4
STRATEGY_NAME = "zero_control"
STRATEGY_VERSION = "empty-0"


def load_loop_config(path: str | Path) -> dict:
    """读取 YAML。必须包含地图、时间步、总帧数、是否渲染。"""
    config_path = Path(path)
    loaded = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError(f"配置必须是映射: {config_path}")
    missing = [key for key in ("map", "fixed_delta_seconds", "frame_count", "render") if key not in loaded]
    if missing:
        raise ValueError(f"配置缺少字段: {missing}")
    frame_count = int(loaded["frame_count"])
    timestep = float(loaded["fixed_delta_seconds"])
    render = loaded["render"]
    if frame_count <= 0:
        raise ValueError("frame_count 必须大于 0")
    if timestep <= 0.0:
        raise ValueError("fixed_delta_seconds 必须大于 0")
    if not isinstance(render, bool):
        raise ValueError("render 必须是 true 或 false")
    return {
        "map": str(loaded["map"]),
        "fixed_delta_seconds": timestep,
        "frame_count": frame_count,
        "render": render,
        "host": str(loaded.get("host", "localhost")),
        "port": int(loaded.get("port", 2000)),
        "tm_port": int(loaded.get("tm_port", TM_PORT)),
        "role_name": str(loaded.get("role_name", ROLE_NAME)),
        "vehicle_blueprint": str(loaded.get("vehicle_blueprint", VEHICLE_BLUEPRINT)),
        "spawn_index": int(loaded.get("spawn_index", SPAWN_INDEX)),
        "python_seed": int(loaded.get("python_seed", 0)),
        "numpy_seed": int(loaded.get("numpy_seed", 0)),
        "traffic_manager_seed": int(loaded.get("traffic_manager_seed", 0)),
        "path": config_path,
    }


def check_frame_clock(records: list[FrameRecord], timestep_s: float, tolerance_s: float = CLOCK_TOLERANCE_S) -> None:
    """帧号从 0 连续，仿真时间等于帧号乘时间步。"""
    if not records:
        raise AssertionError("没有帧")
    for index, record in enumerate(records):
        if record.frame != index:
            raise AssertionError(f"帧号不连续: 期望 {index}，实际 {record.frame}")
        expected = index * timestep_s
        if abs(record.sim_time_s - expected) > tolerance_s:
            raise AssertionError(
                f"帧 {index} 的仿真时间 {record.sim_time_s} 不等于 {expected}"
            )


def run_empty_control(config: dict, *, run_id: str | None = None, debug_draw: bool = False, run_root: str | Path | None = None) -> dict:
    """跑一轮零控制。返回运行目录、是否被中断、帧数和出生点。"""
    random.seed(config["python_seed"])
    _seed_numpy(config["numpy_seed"])
    timestep = config["fixed_delta_seconds"]
    scene_id = f"{config['map']}_empty"
    resolved_run_id = run_id or _default_run_id(config["map"])
    root = Path(run_root) if run_root is not None else _PROJECT_ROOT / "runs"
    run_dir = root / resolved_run_id
    records: list[FrameRecord] = []
    interrupted = False

    with CarlaSession(
        host=config["host"],
        port=config["port"],
        map_name=config["map"],
        fixed_delta_seconds=timestep,
        render=True,
        tm_port=config["tm_port"],
        role_name=config["role_name"],
        vehicle_blueprint=config["vehicle_blueprint"],
        spawn_index=config["spawn_index"],
        traffic_manager_seed=config["traffic_manager_seed"],
        debug_draw=debug_draw,
    ) as session:
        manifest = build_manifest(
            run_id=resolved_run_id,
            scene_id=scene_id,
            strategy_name=STRATEGY_NAME,
            strategy_version=STRATEGY_VERSION,
            strategy_config_path=config["path"],
            carla_version=session.server_version,
            map_name=config["map"],
            fixed_delta_seconds=timestep,
            synchronous_mode=True,
            physics_substep=session.physics_substep(),
            python_seed=config["python_seed"],
            numpy_seed=config["numpy_seed"],
            traffic_manager_seed=config["traffic_manager_seed"],
            weather=session.weather_dict(),
        )
        manifest["ego_spawn"] = dict(session.spawn_pose or {})
        manifest["vehicle_blueprint"] = config["vehicle_blueprint"]
        manifest["role_name"] = config["role_name"]
        manifest["spawn_index"] = config["spawn_index"]
        with RunLogger(run_dir, manifest) as logger:
            baseline_elapsed = None
            collision_count = 0
            try:
                for index in range(config["frame_count"]):
                    # 1. 读取当前帧快照
                    before = read_snapshot(session)
                    _follow_ego(session)
                    # 2. 推进一帧（控制量已在 advance 里写成零）
                    tick_frame = advance(session)
                    # 3. 收集车辆响应与事件
                    sample = collect(session, tick_frame)
                    if sample["carla_frame"] < int(before.timestamp.frame):
                        raise RuntimeError("world.tick() 没有推进 CARLA 帧号")
                    if baseline_elapsed is None:
                        baseline_elapsed = sample["elapsed_seconds"]
                    sim_time_s = sample["elapsed_seconds"] - baseline_elapsed
                    # 4. 写日志
                    record = _frame_record(
                        run_id=resolved_run_id,
                        scene_id=scene_id,
                        index=index,
                        sim_time_s=sim_time_s,
                        sample=sample,
                    )
                    write_log(logger, record, sample["collisions"], sim_time_s)
                    collision_count += len(sample["collisions"])
                    records.append(record)
            except KeyboardInterrupt:
                interrupted = True
            logger.write_summary(
                _summary(resolved_run_id, scene_id, records, interrupted, collision_count > 0)
            )

    if records and not interrupted:
        check_frame_clock(records, timestep)
        line_count = (run_dir / FRAMES_NAME).read_bytes().count(b"\n")
        if line_count != len(records):
            raise RuntimeError(f"frames.jsonl 有 {line_count} 行，帧数是 {len(records)}")
    if server_synchronous_mode(config["host"], config["port"]):
        raise RuntimeError("退出后服务端仍停在同步模式")
    return {
        "run_dir": run_dir,
        "interrupted": interrupted,
        "frame_count": len(records),
        "spawn": dict(manifest["ego_spawn"]),
        "scene_id": scene_id,
    }


def _follow_ego(session: CarlaSession) -> None:
    if session.ego is None or session.world is None:
        return
    from sim.carla_stage import follow_spectator

    transform = session.ego.get_transform()
    follow_spectator(
        session.world,
        float(transform.location.x),
        float(transform.location.y),
        float(transform.location.z),
        math.radians(float(transform.rotation.yaw)),
    )


def read_snapshot(session: CarlaSession):
    return session.read_snapshot()


def advance(session: CarlaSession) -> int:
    return session.advance()


def collect(session: CarlaSession, tick_frame: int) -> dict:
    return session.collect(tick_frame)


def write_log(logger: RunLogger, record: FrameRecord, collisions: list, sim_time_s: float) -> None:
    logger.log_frame(record)
    for event in collisions:
        logger.log_event("collision", sim_time_s, _collision_detail(event))


def _frame_record(run_id: str, scene_id: str, index: int, sim_time_s: float, sample: dict) -> FrameRecord:
    flags = [] if sample["imu_ok"] else ["imu_missing"]
    return FrameRecord(
        run_id=run_id,
        scene_id=scene_id,
        strategy_name=STRATEGY_NAME,
        frame=index,
        sim_time_s=float(sim_time_s),
        source_tag="gt",
        ego=sample["ego"],
        threat=NO_THREAT,
        aeb_state=AEB_IDLE,
        aes_state=AES_IDLE,
        trigger_reason=None,
        exit_reason=None,
        arbitration_reason=None,
        control_request=ZERO_REQUEST,
        actual_control=ZERO_APPLIED,
        quality_flags=flags,
    )


def _summary(
    run_id: str,
    scene_id: str,
    records: list[FrameRecord],
    interrupted: bool,
    collided: bool,
) -> EpisodeSummary:
    if interrupted:
        outcome = "incomplete"
    elif collided:
        outcome = "collision"
    else:
        outcome = "avoided"
    if not records:
        return EpisodeSummary(
            run_id=run_id,
            scene_id=scene_id,
            strategy_name=STRATEGY_NAME,
            outcome=outcome,
            collided=collided,
            min_distance_m=None,
            min_ttc_s=None,
            first_trigger_time_s=None,
            aeb_first_trigger_time_s=None,
            aes_first_trigger_time_s=None,
            peak_decel_mps2=None,
            speed_drop_mps=None,
            initial_speed_mps=None,
            final_speed_mps=None,
            duration_s=0.0,
            frame_count=0,
            invalid_frame_count=0,
        )
    speeds = [record.ego.speed_mps for record in records]
    decels = [-record.ego.a_long_mps2 for record in records]
    return EpisodeSummary(
        run_id=run_id,
        scene_id=scene_id,
        strategy_name=STRATEGY_NAME,
        outcome=outcome,
        collided=collided,
        min_distance_m=None,
        min_ttc_s=None,
        first_trigger_time_s=None,
        aeb_first_trigger_time_s=None,
        aes_first_trigger_time_s=None,
        peak_decel_mps2=max(decels),
        speed_drop_mps=None,
        initial_speed_mps=speeds[0],
        final_speed_mps=speeds[-1],
        duration_s=records[-1].sim_time_s - records[0].sim_time_s,
        frame_count=len(records),
        invalid_frame_count=sum(record.threat.invalid_reason is not None for record in records),
    )


def _collision_detail(event) -> dict:
    other = event.other_actor
    impulse = event.normal_impulse
    return {
        "other_id": int(other.id),
        "other_type": str(other.type_id),
        "impulse_x_ns": float(impulse.x),
        "impulse_y_ns": float(impulse.y),
        "impulse_z_ns": float(impulse.z),
    }


def _seed_numpy(seed: int) -> None:
    """记录进清单的 numpy 种子。内存不够时跳过，不让数值库把进程打退出去。"""
    try:
        import numpy as np
    except Exception:
        return
    np.random.seed(seed)


def _default_run_id(map_name: str) -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    return f"empty_{map_name}_{stamp}"


def _install_break_handler() -> None:
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, _raise_keyboard_interrupt)


def _raise_keyboard_interrupt(_signum, _frame) -> None:
    raise KeyboardInterrupt


def main(argv: list[str] | None = None) -> int:
    _install_break_handler()
    parser = argparse.ArgumentParser(description="CARLA 同步空控制循环。控制量恒为零。")
    parser.add_argument("--config", required=True, help="YAML 配置，含地图、时间步、总帧数、是否渲染")
    parser.add_argument("--debug-draw", action=argparse.BooleanOptionalAction, default=True, help="用 world.debug 画自车包围盒和速度，只保留一拍")
    parser.add_argument("--run-id", default=None, help="运行目录名，默认按时间和地图生成")
    parser.add_argument("--frames", type=int, default=None, help="覆盖配置里的总帧数")
    args = parser.parse_args(argv)
    config = load_loop_config(args.config)
    if args.frames is not None:
        if args.frames <= 0:
            raise SystemExit("--frames 必须大于 0")
        config["frame_count"] = args.frames

    result = run_empty_control(config, run_id=args.run_id, debug_draw=args.debug_draw)
    run_dir = result["run_dir"]
    try:
        from viz.ego_track import plot_ego_run
        from viz.replay_bev import export_bev_replay

        figure = plot_ego_run(run_dir)
        animation = export_bev_replay(run_dir)
    except Exception as exc:
        print(f"出图失败: {type(exc).__name__}: {exc}", file=sys.stderr)
        print(f"run_dir={run_dir}")
        return 1
    spawn = result["spawn"]
    print(f"run_dir={run_dir}")
    print(f"frames={result['frame_count']}")
    print(
        "spawn_m="
        f"{spawn.get('x_m'):.6f},{spawn.get('y_m'):.6f},{spawn.get('z_m'):.6f}"
    )
    print(f"figure={figure}")
    print(f"replay={animation}")
    if result["interrupted"]:
        print("interrupted=1")
        return 130
    return 0


def reload_records(run_dir: str | Path) -> list[FrameRecord]:
    raw = (Path(run_dir) / FRAMES_NAME).read_bytes()
    return [parse_line(FrameRecord, line.decode("utf-8")) for line in raw.splitlines(keepends=True)]


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
