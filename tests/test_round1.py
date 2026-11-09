"""第 1 轮：100 帧假数据的字节级往返、清单字段和总览图。不导入 carla。"""

from __future__ import annotations

import json
import platform
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import matplotlib

matplotlib.use("Agg", force=True)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from experiment import build_manifest, file_sha256, git_head
from run_logger import RUN_FILES, RunLogger
from schemas import (
    AEB_EXIT,
    AEB_FULL,
    AEB_IDLE,
    AEB_PARTIAL,
    AEB_WARN,
    AES_IDLE,
    ActualControl,
    ControlRequest,
    EgoState,
    EpisodeSummary,
    FrameRecord,
    ThreatResult,
    Track,
    dump_line,
    parse_line,
)
from viz.common import load_run, plot_run_overview


RUN_DIR = ROOT / "runs" / "round1_fake100"
CONFIG_PATH = ROOT / "configs" / "demo_strategy.yaml"
SCENE_ID = "demo_ccrs"
RUN_ID = "round1_fake100"
DT_S = 0.05


class Round1ContractTest(unittest.TestCase):
    def test_sources_do_not_import_carla(self) -> None:
        for relative in ("schemas.py", "run_logger.py", "experiment.py", "viz/common.py", "viz/__init__.py"):
            text = (ROOT / relative).read_text(encoding="utf-8")
            self.assertNotIn("import carla", text)
            self.assertNotIn("from carla", text)
        self.assertNotIn("carla", sys.modules)

    def test_invalid_threat_requires_reason(self) -> None:
        with self.assertRaises(ValueError):
            ThreatResult(
                ttc_s=None,
                pred_min_distance_m=1.0,
                invalid_reason=None,
                threat_id=None,
                in_path=None,
                closing_speed_mps=None,
                overlap_ratio=None,
            )
        with self.assertRaises(ValueError):
            ThreatResult(
                ttc_s=1.5,
                pred_min_distance_m=2.0,
                invalid_reason="should_be_empty",
                threat_id="lead_01",
                in_path=True,
                closing_speed_mps=5.0,
                overlap_ratio=0.8,
            )

    def test_manifest_records_requested_fields(self) -> None:
        with tempfile.TemporaryDirectory() as temp_name:
            config = Path(temp_name) / "strategy.yaml"
            config.write_text("name: aeb_ttc\nttc_full_s: 0.8\n", encoding="utf-8")
            checksum = file_sha256(config)
            manifest = build_manifest(
                run_id="manifest_case",
                scene_id="case_01",
                strategy_name="aeb",
                strategy_version="0.1.0",
                strategy_config_path=config,
                carla_version=None,
                map_name="Town03",
                fixed_delta_seconds=0.05,
                synchronous_mode=True,
                physics_substep={
                    "substepping": True,
                    "max_substep_delta_time_s": 0.01,
                    "max_substeps": 10,
                },
                python_seed=1,
                numpy_seed=2,
                traffic_manager_seed=3,
                weather={"cloudiness": 10.0, "precipitation": 0.0},
                code_version="abc123",
            )
        self.assertEqual(manifest["carla_version"], None)
        self.assertEqual(manifest["python_version"], platform.python_version())
        self.assertEqual(manifest["strategy_version"], "0.1.0")
        self.assertEqual(manifest["map"], "Town03")
        self.assertEqual(manifest["fixed_delta_seconds"], 0.05)
        self.assertEqual(manifest["synchronous_mode"], True)
        self.assertEqual(manifest["physics_substep"]["max_substeps"], 10)
        self.assertEqual(manifest["physics_substep"]["max_substep_delta_time_s"], 0.01)
        self.assertEqual(manifest["seeds"]["python"], 1)
        self.assertEqual(manifest["seeds"]["numpy"], 2)
        self.assertEqual(manifest["seeds"]["traffic_manager"], 3)
        self.assertEqual(manifest["weather"]["cloudiness"], 10.0)
        self.assertEqual(manifest["strategy_config"]["ttc_full_s"], 0.8)
        self.assertEqual(manifest["strategy_config_sha256"], checksum)
        self.assertEqual(manifest["code_version"], "abc123")

    def test_git_hash_is_null_when_project_is_not_its_own_repo(self) -> None:
        commit = git_head(ROOT)
        self.assertTrue(commit is None or (len(commit) == 40 and all(ch in "0123456789abcdef" for ch in commit)))

    def test_hundred_frames_roundtrip_and_overview(self) -> None:
        if RUN_DIR.exists():
            shutil.rmtree(RUN_DIR)
        frames = _fake_frames()
        self.assertEqual(len(frames), 100)
        manifest = build_manifest(
            run_id=RUN_ID,
            scene_id=SCENE_ID,
            strategy_name="aeb",
            strategy_version="0.1.0",
            strategy_config_path=CONFIG_PATH,
            carla_version=None,
            map_name="Town03",
            fixed_delta_seconds=DT_S,
            synchronous_mode=True,
            physics_substep={
                "substepping": True,
                "max_substep_delta_time_s": 0.01,
                "max_substeps": 10,
            },
            python_seed=1,
            numpy_seed=2,
            traffic_manager_seed=3,
            weather={
                "cloudiness": 10.0,
                "precipitation": 0.0,
                "sun_altitude_angle": 45.0,
                "fog_density": 0.0,
            },
        )

        with RunLogger(RUN_DIR, manifest) as logger:
            for index, frame in enumerate(frames):
                logger.log_frame(frame)
                logger.log_target(frame.frame, frame.sim_time_s, _fake_track(frame))
                if index == 49:
                    prefix = (RUN_DIR / "frames.jsonl").read_bytes()
                    self.assertEqual(prefix.count(b"\n"), 50)
            for event_frame, event_type in ((20, "aeb_warn"), (40, "aeb_partial"), (70, "aeb_full"), (90, "aeb_exit")):
                logger.log_event(
                    event_type,
                    frames[event_frame].sim_time_s,
                    {"reason": frames[event_frame].trigger_reason or frames[event_frame].exit_reason},
                )
            before_summary = (RUN_DIR / "frames.jsonl").read_bytes()
            self.assertTrue(before_summary.startswith(prefix))
            logger.write_summary(_summary(frames))
            after_summary = (RUN_DIR / "frames.jsonl").read_bytes()
            self.assertEqual(after_summary, before_summary)

        original = (RUN_DIR / "frames.jsonl").read_bytes()
        rebuilt = bytearray()
        restored = []
        for line in original.splitlines(keepends=True):
            record = parse_line(FrameRecord, line.decode("utf-8"))
            restored.append(record)
            rebuilt.extend(dump_line(record).encode("utf-8"))
        self.assertEqual(bytes(rebuilt), original)
        self.assertEqual(restored, frames)

        self.assertIsNone(restored[0].threat.ttc_s)
        self.assertIsNone(restored[0].threat.pred_min_distance_m)
        self.assertEqual(restored[0].threat.invalid_reason, "no_threat")
        self.assertIsNone(restored[0].actual_control.desired_speed_mps)
        self.assertIsNone(restored[0].ego.steer_road_rad)
        self.assertIsNotNone(restored[30].threat.ttc_s)
        self.assertIsNone(restored[30].threat.pred_min_distance_m)
        self.assertEqual(restored[30].threat.invalid_reason, "no_overlap_estimate")
        self.assertIsNone(restored[50].threat.invalid_reason)
        self.assertIsNone(restored[50].actual_control.desired_speed_mps)
        self.assertIsNotNone(restored[50].control_request.desired_speed_mps)

        first = json.loads(original.splitlines()[0])
        active = json.loads(original.splitlines()[50])
        self.assertIn("ttc_s", first["threat"])
        self.assertIsNone(first["threat"]["ttc_s"])
        self.assertNotEqual(active["control_request"]["brake"], active["actual_control"]["brake"])

        for name in RUN_FILES:
            path = RUN_DIR / name
            self.assertTrue(path.is_file(), name)
            self.assertGreater(path.stat().st_size, 0, name)

        manifest_text = (RUN_DIR / "manifest.json").read_text(encoding="utf-8")
        self.assertIn('"carla_version": null', manifest_text)
        self.assertIn(platform.python_version(), manifest_text)
        if git_head(ROOT) is None:
            self.assertIn('"code_version": null', manifest_text)
        else:
            self.assertIn(git_head(ROOT), manifest_text)

        loaded = load_run(RUN_DIR)
        self.assertEqual(len(loaded.frames), 100)
        self.assertEqual(len(loaded.targets), 100)
        self.assertEqual(len(loaded.events), 4)
        self.assertEqual(loaded.manifest["scene_id"], SCENE_ID)
        self.assertEqual(loaded.summary["frame_count"], 100)
        missing_ttc = loaded.frames.loc[0, "threat_ttc_s"]
        self.assertTrue(missing_ttc is None or missing_ttc != missing_ttc)
        self.assertNotEqual(missing_ttc, 0)
        self.assertFalse(
            loaded.frames["control_request_brake"].equals(loaded.frames["actual_control_brake"])
        )

        figure_path = plot_run_overview(RUN_DIR)
        self.assertEqual(figure_path.name, f"{SCENE_ID}_overview.png")
        self.assertTrue(figure_path.is_file())
        self.assertTrue(figure_path.read_bytes().startswith(b"\x89PNG"))
        self.assertGreater(figure_path.stat().st_size, 1000)


def _fake_frames() -> list[FrameRecord]:
    frames = []
    speed = 20.0
    position = 0.0
    for index in range(100):
        state, accel, request_brake, actual_brake = _phase(index)
        if index < 20 or index >= 90:
            ttc = None
            gap = None
            reason = "no_threat" if index < 20 else "maneuver_ended"
            threat_id = None
            in_path = None
            closing = None
            overlap = None
        elif index == 30:
            ttc = 3.2
            gap = None
            reason = "no_overlap_estimate"
            threat_id = "lead_01"
            in_path = True
            closing = 5.0
            overlap = None
        else:
            ttc = max(0.4, 4.0 - (index - 20) * 0.05)
            gap = 12.0 - (index - 20) * 0.08
            reason = None
            threat_id = "lead_01"
            in_path = True
            closing = 5.0
            overlap = 0.8
        trigger = {20: "ttc_below_warn", 40: "ttc_below_partial", 70: "ttc_below_full"}.get(index)
        exit_reason = "threat_cleared" if index >= 90 else None
        arbitration = "aeb_outranks_aes" if 70 <= index < 90 else None
        desired = 12.0 if index >= 40 else None
        frame = FrameRecord(
            run_id=RUN_ID,
            scene_id=SCENE_ID,
            strategy_name="aeb",
            frame=index,
            sim_time_s=index * DT_S,
            source_tag="gt",
            ego=EgoState(
                x_m=position,
                y_m=0.0,
                z_m=0.0,
                roll_rad=0.0,
                pitch_rad=0.0,
                yaw_rad=0.0,
                v_long_mps=speed,
                v_lat_mps=0.0,
                v_up_mps=0.0,
                speed_mps=speed,
                yaw_rate_radps=0.0,
                a_long_mps2=accel,
                a_lat_mps2=0.0,
                steer_road_rad=None,
            ),
            threat=ThreatResult(
                ttc_s=ttc,
                pred_min_distance_m=gap,
                invalid_reason=reason,
                threat_id=threat_id,
                in_path=in_path,
                closing_speed_mps=closing,
                overlap_ratio=overlap,
            ),
            aeb_state=state,
            aes_state=AES_IDLE,
            trigger_reason=trigger,
            exit_reason=exit_reason,
            arbitration_reason=arbitration,
            control_request=ControlRequest(0.0, request_brake, 0.0, desired),
            actual_control=ActualControl(0.0, actual_brake, 0.0, None),
            quality_flags=["ttc_invalid"] if reason else [],
        )
        frames.append(frame)
        speed = max(0.0, speed + accel * DT_S)
        position += speed * DT_S
    return frames


def _phase(index: int) -> tuple[str, float, float, float]:
    if index < 20:
        return AEB_IDLE, 0.0, 0.0, 0.0
    if index < 40:
        return AEB_WARN, 0.0, 0.0, 0.0
    if index < 70:
        return AEB_PARTIAL, -4.0, 0.4, 0.25
    if index < 90:
        return AEB_FULL, -6.0, 1.0, 0.8
    return AEB_EXIT, 0.0, 0.0, 0.0


def _fake_track(frame: FrameRecord) -> Track:
    return Track(
        track_id="lead_01",
        source="gt",
        class_name="vehicle",
        x_m=frame.ego.x_m + 25.0,
        y_m=0.0,
        z_m=0.0,
        yaw_rad=0.0,
        v_long_mps=15.0,
        v_lat_mps=0.0,
        speed_mps=15.0,
        length_m=4.5,
        width_m=1.8,
        height_m=1.5,
        rel_x_m=25.0,
        rel_y_m=0.0,
        rel_v_long_mps=15.0 - frame.ego.v_long_mps,
        rel_v_lat_mps=0.0,
        in_path=True,
    )


def _summary(frames: list[FrameRecord]) -> EpisodeSummary:
    ttcs = [frame.threat.ttc_s for frame in frames if frame.threat.ttc_s is not None]
    gaps = [frame.threat.pred_min_distance_m for frame in frames if frame.threat.pred_min_distance_m is not None]
    speeds = [frame.ego.speed_mps for frame in frames]
    first_trigger = next((frame.sim_time_s for frame in frames if frame.aeb_state != AEB_IDLE), None)
    return EpisodeSummary(
        run_id=RUN_ID,
        scene_id=SCENE_ID,
        strategy_name="aeb",
        outcome="avoided",
        collided=False,
        min_distance_m=_round(min(gaps) if gaps else None),
        min_ttc_s=_round(min(ttcs) if ttcs else None),
        first_trigger_time_s=_round(first_trigger),
        aeb_first_trigger_time_s=_round(first_trigger),
        aes_first_trigger_time_s=None,
        peak_decel_mps2=_round(max(-frame.ego.a_long_mps2 for frame in frames)),
        speed_drop_mps=_round(max(speeds) - min(speeds)),
        initial_speed_mps=_round(speeds[0]),
        final_speed_mps=_round(speeds[-1]),
        duration_s=_round(frames[-1].sim_time_s - frames[0].sim_time_s),
        frame_count=len(frames),
        invalid_frame_count=sum(frame.threat.invalid_reason is not None for frame in frames),
    )


def _round(value: float | None) -> float | None:
    if value is None:
        return None
    return round(float(value), 6)


if __name__ == "__main__":
    unittest.main()
