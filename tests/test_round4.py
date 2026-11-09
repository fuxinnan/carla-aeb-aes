"""第 4 轮：五个手算 TTC、真值航迹，以及横穿无干预时 TTC 单调降到 0。不启动 CARLA。"""

from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

import matplotlib

matplotlib.use("Agg", force=True)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from experiment import build_manifest
from perception.tracker import GroundTruthTracker, TargetTruth
from perception.visibility import VisibilityModel
from run_logger import RunLogger
from scenarios.geometry import make_straight_road
from scenarios.motion import rollout
from scenarios.spec import load_spec
from schemas import (
    AEB_IDLE,
    AES_IDLE,
    ActualControl,
    EgoState,
    EpisodeSummary,
    FrameRecord,
    ThreatResult,
    load_dataclass,
)
from strategies.no_intervention import NoIntervention
from threat.assessor import PREDICTION, ThreatAssessor
from threat.pipeline import assess_frame
from viz.threat import plot_threat_snapshot, plot_threat_timeline
from viz.unit_cases import save_unit_case


PED_YAML = ROOT / "configs" / "scenarios" / "pedestrian_crossing_collision.yaml"
UNIT_DIR = ROOT / "figures" / "unit_cases"
MAX_DECEL_MPS2 = 5.0


class Round4ThreatTest(unittest.TestCase):
    def test_sources_do_not_import_carla(self) -> None:
        for relative in (
            "perception/tracker.py",
            "perception/visibility.py",
            "threat/assessor.py",
            "threat/pipeline.py",
            "strategies/no_intervention.py",
            "viz/threat.py",
            "viz/unit_cases.py",
        ):
            text = (ROOT / relative).read_text(encoding="utf-8")
            self.assertNotIn("import carla", text)
            self.assertNotIn("from carla", text)

    def test_old_threat_record_still_loads(self) -> None:
        loaded = load_dataclass(
            ThreatResult,
            {
                "ttc_s": None,
                "pred_min_distance_m": None,
                "invalid_reason": "no_threat",
                "threat_id": None,
                "in_path": None,
                "closing_speed_mps": None,
                "overlap_ratio": None,
            },
        )
        self.assertIsNone(loaded.ttc_s)
        self.assertIsNone(loaded.conflict_x_m)
        self.assertIsNone(loaded.brake_avoidable)
        self.assertIsNone(loaded.prediction)

    def test_tracker_aligns_timestamp_and_visibility_keeps_every_track(self) -> None:
        ego = _ego({"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "v_long_mps": 8.0, "length_m": 4.0, "width_m": 2.0})
        first = _truth("a", {"x_m": 12.0, "y_m": 0.0, "yaw_rad": 0.0, "v_long_mps": 0.0, "length_m": 2.0, "width_m": 2.0}, 1.0)
        second = _truth("b", {"x_m": 0.0, "y_m": 6.0, "yaw_rad": math.pi / 2.0, "v_long_mps": 1.0, "length_m": 1.0, "width_m": 1.0}, 1.0)
        tracks = GroundTruthTracker().update(1.0, ego, [second, first], ego_length_m=4.0, ego_width_m=2.0)
        self.assertEqual([item.track_id for item in tracks], ["a", "b"])
        self.assertTrue(all(item.timestamp_s == 1.0 and item.source == "gt" for item in tracks))
        self.assertTrue(tracks[0].in_path)
        self.assertFalse(tracks[1].in_path)
        visible = VisibilityModel().apply(tracks)
        self.assertEqual(visible, tracks)
        self.assertEqual(VisibilityModel().occlusion_ratio(tracks[0]), 0.0)
        stale = _truth("a", {"x_m": 12.0, "y_m": 0.0, "yaw_rad": 0.0, "v_long_mps": 0.0, "length_m": 2.0, "width_m": 2.0}, 0.0)
        with self.assertRaises(ValueError):
            GroundTruthTracker().update(1.0, ego, [stale], ego_length_m=4.0, ego_width_m=2.0)

    def test_no_intervention_holds_speed_without_brake_or_steer(self) -> None:
        request = NoIntervention(10.0).decide(None, [], None)
        self.assertEqual(request.target_decel_mps2, 0.0)
        self.assertEqual(request.throttle, 0.0)
        self.assertEqual(request.brake, 0.0)
        self.assertEqual(request.steer, 0.0)
        self.assertEqual(request.desired_speed_mps, 10.0)
        self.assertEqual(NoIntervention.version, "round4-0")

    def test_five_hand_cases_match_and_figures_show_both_ttc(self) -> None:
        """五个用例的期望 TTC 都按匀速、不转向手算。最大减速度 5 m/s^2，刹停距离 10 m。"""
        for case in _hand_cases():
            _track, threat, request = _evaluate(case["ego"], case["target"])
            self.assertEqual(request.brake, 0.0, case["name"])
            self.assertEqual(request.steer, 0.0, case["name"])
            self.assertEqual(threat.prediction, PREDICTION, case["name"])
            if case["ttc_s"] is None:
                self.assertIsNone(threat.ttc_s, case["name"])
                self.assertEqual(threat.invalid_reason, case["reason"], case["name"])
                self.assertIsNone(threat.conflict_x_m, case["name"])
            else:
                self.assertIsNone(threat.invalid_reason, case["name"])
                self.assertAlmostEqual(threat.ttc_s, case["ttc_s"], places=4, msg=case["name"])
                self.assertAlmostEqual(threat.pred_min_distance_m, 0.0, places=6, msg=case["name"])
                self.assertAlmostEqual(threat.conflict_x_m, case["conflict"][0], places=4, msg=case["name"])
                self.assertAlmostEqual(threat.conflict_y_m, case["conflict"][1], places=4, msg=case["name"])
            if case.get("gap_m") is not None:
                self.assertAlmostEqual(threat.pred_min_distance_m, case["gap_m"], places=4, msg=case["name"])
            self.assertEqual(threat.brake_avoidable, case["brake_avoidable"], case["name"])
            if case["ttc_s"] is None:
                self.assertNotEqual(threat.ttc_s, 0.0, case["name"])
            figure = save_unit_case(
                UNIT_DIR / case["file"],
                title=case["title"],
                ego=case["ego"],
                target=case["target"],
                expected_ttc_s=case["ttc_s"],
                actual_ttc_s=threat.ttc_s,
                expected_reason=case["reason"],
                actual_reason=threat.invalid_reason,
                conflict_xy=None if threat.conflict_x_m is None else (threat.conflict_x_m, threat.conflict_y_m),
            )
            self.assertTrue(figure.is_file(), figure)
            self.assertGreater(figure.stat().st_size, 1000, figure)
            self.assertTrue(figure.read_bytes().startswith(b"\x89PNG"), figure)

    def test_crossing_ttc_decreases_to_zero_and_passed_conflict_stays_invalid(self) -> None:
        spec = load_spec(PED_YAML)
        road = make_straight_road(length_m=160.0, step_m=1.0, yaw_rad=0.0)
        plan = rollout(spec, road)
        self.assertIsNotNone(plan.collision_frame)
        tracker = GroundTruthTracker()
        visibility = VisibilityModel()
        assessor = ThreatAssessor(max_decel_mps2=MAX_DECEL_MPS2)
        policy = NoIntervention()
        logged = []
        valid = []
        seen_valid = False
        for step in plan.steps:
            ego_body = {
                "x_m": step.ego.x_m,
                "y_m": step.ego.y_m,
                "yaw_rad": step.ego.yaw_rad,
                "v_long_mps": step.ego.speed_mps,
                "length_m": spec.ego_length_m,
                "width_m": spec.ego_width_m,
            }
            target_body = {
                "x_m": step.target.x_m,
                "y_m": step.target.y_m,
                "yaw_rad": step.target.yaw_rad,
                "v_long_mps": step.target.speed_mps,
                "length_m": spec.target_length_m,
                "width_m": spec.target_width_m,
                "track_id": "ped_01",
                "class_name": "pedestrian",
            }
            track, threat, request = _evaluate(
                ego_body,
                target_body,
                timestamp_s=step.sim_time_s,
                tracker=tracker,
                visibility=visibility,
                assessor=assessor,
                policy=policy,
            )
            self.assertEqual(request.brake, 0.0)
            self.assertEqual(request.steer, 0.0)
            self.assertEqual(threat.prediction, PREDICTION)
            if threat.ttc_s is None:
                self.assertTrue(threat.invalid_reason)
                if seen_valid:
                    self.fail(f"有效 TTC 在帧 {step.frame} 中断：{threat.invalid_reason}")
            else:
                seen_valid = True
                valid.append((step.frame, threat.ttc_s))
            logged.append((_logged_frame(spec, step, _ego_state(ego_body), threat, request), track))
        self.assertGreater(len(valid), 5)
        increases = [(left, right) for left, right in zip(valid, valid[1:]) if right[1] > left[1] + 1e-6]
        self.assertEqual(increases, [], f"TTC 出现回升：{increases[:6]}")
        self.assertEqual(valid[-1][0], plan.collision_frame)
        self.assertLessEqual(valid[-1][1], 0.05)
        self.assertLess(valid[-2][1], 0.2)
        passed = _evaluate(
            {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "v_long_mps": 10.0, "length_m": 4.0, "width_m": 2.0},
            {
                "x_m": 15.0,
                "y_m": 6.0,
                "yaw_rad": math.pi / 2.0,
                "v_long_mps": 2.0,
                "length_m": 2.0,
                "width_m": 2.0,
                "track_id": "cross_01",
                "class_name": "vehicle",
            },
        )[1]
        self.assertIsNone(passed.ttc_s)
        self.assertEqual(passed.invalid_reason, "passed_conflict")
        for extra in (0.5, 1.0, 1.5):
            later = _evaluate(
                {"x_m": 10.0 * extra, "y_m": 0.0, "yaw_rad": 0.0, "v_long_mps": 10.0, "length_m": 4.0, "width_m": 2.0},
                {
                    "x_m": 15.0,
                    "y_m": 6.0 + 2.0 * extra,
                    "yaw_rad": math.pi / 2.0,
                    "v_long_mps": 2.0,
                    "length_m": 2.0,
                    "width_m": 2.0,
                    "track_id": "cross_01",
                    "class_name": "vehicle",
                },
                timestamp_s=extra,
            )[1]
            self.assertIsNone(later.ttc_s)
            self.assertEqual(later.invalid_reason, "passed_conflict")
        with tempfile.TemporaryDirectory() as temp_name:
            run_dir = Path(temp_name) / "crossing"
            _write_run(spec, logged, run_dir)
            timeline = plot_threat_timeline(run_dir)
            snapshot = plot_threat_snapshot(run_dir)
            for path in (timeline, snapshot):
                self.assertTrue(path.is_file(), path)
                self.assertTrue(path.read_bytes().startswith(b"\x89PNG"), path)
                self.assertGreater(path.stat().st_size, 1000, path)


def _hand_cases() -> list[dict]:
    ego = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "v_long_mps": 10.0, "length_m": 4.0, "width_m": 2.0}
    # 静止障碍：中心距 8 m，半长 2 m 和 1 m，保险杠间距 5 m。TTC = 5 / 10 = 0.5 s。
    # 冲突点在 t=0.5 的车头，x = 5 + 2 = 7。刹停距离 10 m > 5 m。
    stationary = {
        "x_m": 8.0,
        "y_m": 0.0,
        "yaw_rad": 0.0,
        "v_long_mps": 0.0,
        "length_m": 2.0,
        "width_m": 2.0,
    }
    # 前车等速：中心距 20 m，间距 17 m，靠近速度 0。TTC 不能写成 0。
    same_speed = {
        "x_m": 20.0,
        "y_m": 0.0,
        "yaw_rad": 0.0,
        "v_long_mps": 10.0,
        "length_m": 2.0,
        "width_m": 2.0,
    }
    # 横穿恰好相撞：目标中心 (23, -4)，沿 +Y 以 2 m/s 走。
    # X 向第一次接触 t = (23 - 1 - 2) / 10 = 2 s。Y 向重叠是 t ∈ [1, 3]。所以 TTC = 2 s。
    # 接触点 x = 22。车头现在在 x = 2，制动距离 20 m，刹停距离 10 m，来得及。
    crossing_hit = {
        "x_m": 23.0,
        "y_m": -4.0,
        "yaw_rad": math.pi / 2.0,
        "v_long_mps": 2.0,
        "length_m": 2.0,
        "width_m": 2.0,
    }
    # 横穿提前通过：同样的横向起点，速度 5 m/s。
    # X 重叠 t ∈ [2.0, 2.6]，Y 重叠 t ∈ [0.4, 1.2]，没有公共时间。
    crossing_early = {
        "x_m": 23.0,
        "y_m": -4.0,
        "yaw_rad": math.pi / 2.0,
        "v_long_mps": 5.0,
        "length_m": 2.0,
        "width_m": 2.0,
    }
    # 目标在自车后方，速度更慢，间距从 12 m 继续变大。
    behind = {
        "x_m": -16.0,
        "y_m": 0.0,
        "yaw_rad": 0.0,
        "v_long_mps": 6.0,
        "length_m": 4.0,
        "width_m": 2.0,
    }
    return [
        _named("01_stationary.png", "静止障碍", ego, stationary, 0.5, None, False, (7.0, 0.0), 0.0),
        _named("02_same_speed.png", "前车等速", ego, same_speed, None, "not_closing", True, None, 17.0),
        _named("03_crossing_hit.png", "横穿恰好相撞", ego, crossing_hit, 2.0, None, True, (22.0, 0.0), 0.0),
        _named("04_crossing_early.png", "横穿提前通过", ego, crossing_early, None, "no_collision_course", True, None, None),
        _named("05_receding_behind.png", "目标从车后驶离", ego, behind, None, "target_behind", True, None, 12.0),
    ]


def _named(file_name, title, ego, target, ttc_s, reason, brake_avoidable, conflict, gap_m) -> dict:
    target = dict(target)
    target.setdefault("track_id", "tgt_01")
    target.setdefault("class_name", "vehicle")
    return {
        "file": file_name,
        "name": title,
        "title": title,
        "ego": ego,
        "target": target,
        "ttc_s": ttc_s,
        "reason": reason,
        "brake_avoidable": brake_avoidable,
        "conflict": conflict,
        "gap_m": gap_m,
    }


def _evaluate(ego_body, target_body, timestamp_s=0.0, tracker=None, visibility=None, assessor=None, policy=None):
    ego = _ego_state(ego_body)
    observation = _truth(target_body.get("track_id", "tgt_01"), target_body, timestamp_s, target_body.get("class_name", "vehicle"))
    tracks, threat, request = assess_frame(
        timestamp_s=timestamp_s,
        ego=ego,
        observations=[observation],
        ego_length_m=ego_body["length_m"],
        ego_width_m=ego_body["width_m"],
        cruise_speed_mps=abs(ego_body["v_long_mps"]),
        tracker=tracker or GroundTruthTracker(),
        visibility=visibility or VisibilityModel(),
        assessor=assessor or ThreatAssessor(max_decel_mps2=MAX_DECEL_MPS2),
        policy=policy or NoIntervention(),
    )
    return tracks[0], threat, request


def _ego_state(body: dict) -> EgoState:
    speed = math.hypot(body["v_long_mps"], body.get("v_lat_mps", 0.0))
    return EgoState(
        x_m=body["x_m"],
        y_m=body["y_m"],
        z_m=0.0,
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=body["yaw_rad"],
        v_long_mps=body["v_long_mps"],
        v_lat_mps=body.get("v_lat_mps", 0.0),
        v_up_mps=0.0,
        speed_mps=speed,
        yaw_rate_radps=0.0,
        a_long_mps2=0.0,
        a_lat_mps2=0.0,
        steer_road_rad=None,
    )


def _ego(body: dict) -> EgoState:
    return _ego_state(body)


def _truth(track_id: str, body: dict, timestamp_s: float, class_name: str = "vehicle") -> TargetTruth:
    return TargetTruth(
        track_id=track_id,
        class_name=body.get("class_name", class_name),
        x_m=body["x_m"],
        y_m=body["y_m"],
        z_m=0.0,
        yaw_rad=body["yaw_rad"],
        v_long_mps=body["v_long_mps"],
        v_lat_mps=body.get("v_lat_mps", 0.0),
        speed_mps=math.hypot(body["v_long_mps"], body.get("v_lat_mps", 0.0)),
        length_m=body["length_m"],
        width_m=body["width_m"],
        height_m=body.get("height_m", 1.5),
        timestamp_s=timestamp_s,
    )


def _logged_frame(spec, step, ego, threat, request) -> FrameRecord:
    applied = ActualControl(request.throttle, request.brake, request.steer, request.desired_speed_mps)
    return FrameRecord(
        run_id="crossing_offline",
        scene_id=spec.scene_id,
        strategy_name="no_intervention",
        frame=step.frame,
        sim_time_s=step.sim_time_s,
        source_tag="gt",
        ego=ego,
        threat=threat,
        aeb_state=AEB_IDLE,
        aes_state=AES_IDLE,
        trigger_reason=None,
        exit_reason=None,
        arbitration_reason=None,
        control_request=request,
        actual_control=applied,
        quality_flags=["kinematic_script"],
    )


def _write_run(spec, logged: list[tuple[FrameRecord, object]], run_dir: Path) -> None:
    frames = [item[0] for item in logged]
    manifest = build_manifest(
        run_id="crossing_offline",
        scene_id=spec.scene_id,
        strategy_name="no_intervention",
        strategy_version="round4-0",
        strategy_config_path=spec.config_path,
        carla_version=None,
        map_name=spec.map_name,
        fixed_delta_seconds=spec.fixed_delta_seconds,
        synchronous_mode=True,
        physics_substep={"substepping": False, "max_substep_delta_time_s": None, "max_substeps": None},
        python_seed=spec.seed,
        numpy_seed=spec.seed,
        traffic_manager_seed=spec.seed,
        weather={"cloudiness": 0.0},
    )
    manifest["scenario"] = spec.key_fields()
    ttcs = [frame.threat.ttc_s for frame in frames if frame.threat.ttc_s is not None]
    summary = EpisodeSummary(
        run_id="crossing_offline",
        scene_id=spec.scene_id,
        strategy_name="no_intervention",
        outcome="collision",
        collided=True,
        min_distance_m=0.0,
        min_ttc_s=min(ttcs) if ttcs else None,
        first_trigger_time_s=None,
        aeb_first_trigger_time_s=None,
        aes_first_trigger_time_s=None,
        peak_decel_mps2=0.0,
        speed_drop_mps=None,
        initial_speed_mps=frames[0].ego.speed_mps,
        final_speed_mps=frames[-1].ego.speed_mps,
        duration_s=frames[-1].sim_time_s - frames[0].sim_time_s,
        frame_count=len(frames),
        invalid_frame_count=sum(frame.threat.invalid_reason is not None for frame in frames),
    )
    with RunLogger(run_dir, manifest) as logger:
        logger.log_event("scenario_spec", 0.0, spec.key_fields())
        for frame, track in logged:
            logger.log_frame(frame)
            logger.log_target(frame.frame, frame.sim_time_s, track)
        logger.write_summary(summary)


if __name__ == "__main__":
    unittest.main()
