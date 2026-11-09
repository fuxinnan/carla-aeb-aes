"""第 6 轮：硬负例、只读指标和对照表。不启动 CARLA。"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import matplotlib

matplotlib.use("Agg", force=True)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aeb import build_policy, load_aeb_config
from control.actuator import BrakeActuator
from metrics.aggregate import comparison_markdown, investigation_note, stability_lines
from metrics.episode import evaluate_run
from metrics.report import build_report
from scenarios.closed_loop import simulate_episode
from scenarios.geometry import make_straight_road
from scenarios.spec import load_spec

TUNING_CLEAR = ROOT / "configs" / "scenarios" / "tuning" / "pedestrian_crossing_clear_pass.yaml"
TEST_CLEAR = ROOT / "configs" / "scenarios" / "test" / "pedestrian_crossing_clear_pass.yaml"
ROUND5 = ROOT / "runs" / "round5" / "pedestrian_crossing_collision_state_machine" / "rep_00"


class Round6Test(unittest.TestCase):
    def test_clear_pass_is_hard_negative_and_does_not_brake(self) -> None:
        road = make_straight_road(length_m=220.0, step_m=1.0, yaw_rad=0.0)
        config = load_aeb_config()
        for path in (TUNING_CLEAR, TEST_CLEAR):
            spec = load_spec(path)
            self.assertTrue(spec.hard_negative)
            self.assertEqual(spec.instance, "clear_pass")
            plan_gap = None
            for strategy in ("no_intervention", "fixed_ttc", "state_machine"):
                policy = build_policy(strategy, config, spec.ego_speed_mps)
                actuator = BrakeActuator(config.actuator, spec.fixed_delta_seconds)
                episode = simulate_episode(spec, road, policy, actuator, strategy)
                self.assertFalse(episode.collided, strategy)
                self.assertGreater(min(step.gap_m for step in episode.steps), 1.0)
                for step in episode.steps:
                    self.assertEqual(step.record.aeb_state, "IDLE")
                    self.assertEqual(step.record.control_request.target_decel_mps2, 0.0)
                    self.assertIsNone(step.record.threat.ttc_s)
                plan_gap = min(step.gap_m for step in episode.steps)
            self.assertIsNotNone(plan_gap)

    def test_metrics_read_only_logs_and_ignore_avoidance_as_false_brake(self) -> None:
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            avoided = _write_run(root / "avoided", hard_negative=False, collided=False, decel=6.0, state="FULL")
            clear = _write_run(root / "clear", hard_negative=True, collided=False, decel=0.0, state="IDLE")
            mistake = _write_run(root / "mistake", hard_negative=True, collided=False, decel=6.0, state="FULL")
            avoided_metrics = evaluate_run(avoided)
            clear_metrics = evaluate_run(clear)
            mistake_metrics = evaluate_run(mistake)
        self.assertIsNone(avoided_metrics["false_brake"])
        self.assertFalse(clear_metrics["false_brake"])
        self.assertTrue(mistake_metrics["false_brake"])
        self.assertEqual(mistake_metrics["activation_count"], 1)
        self.assertEqual(clear_metrics["state_switch_count"], 0)
        self.assertEqual(avoided_metrics["request_to_decel_frames"], 1)
        self.assertEqual(avoided_metrics["risk_to_request_frames"], 0)
        self.assertIsNone(avoided_metrics["collision_relative_speed_mps"])
        self.assertGreater(avoided_metrics["peak_jerk_mps3"], 0.0)

    def test_round5_log_keeps_request_delay_and_is_not_a_false_brake(self) -> None:
        metrics = evaluate_run(ROUND5)
        summary = json.loads((ROUND5 / "summary.json").read_text(encoding="utf-8"))
        self.assertFalse(metrics["hard_negative"])
        self.assertFalse(metrics["collided"])
        self.assertIsNone(metrics["false_brake"])
        self.assertEqual(metrics["request_to_decel_frames"], summary["brake_delay_frames"])
        self.assertEqual(metrics["request_to_decel_frames"], 1)
        self.assertGreater(metrics["activation_count"], 0)
        self.assertGreater(metrics["risk_to_request_frames"], 0)

    def test_repeat_table_shows_stability_and_report_is_one_file(self) -> None:
        rows = [
            _row("lead_vehicle_brake_collision", "fixed_ttc", 0, collided=False, delay=2),
            _row("lead_vehicle_brake_collision", "fixed_ttc", 1, collided=False, delay=2),
            _row("lead_vehicle_brake_collision", "state_machine", 0, collided=True, speed=3.7, delay=6),
            _row("lead_vehicle_brake_collision", "state_machine", 1, collided=False, delay=6),
            _row("pedestrian_crossing_clear_pass", "fixed_ttc", 0, hard_negative=True, collided=False, delay=None),
            _row("pedestrian_crossing_clear_pass", "state_machine", 0, hard_negative=True, collided=False, delay=None),
            _row("pedestrian_crossing_clear_pass", "no_intervention", 0, hard_negative=True, collided=False, delay=None),
        ]
        text = comparison_markdown(rows)
        self.assertIn("碰撞率", text)
        self.assertIn("硬负例误制动率", text)
        self.assertNotIn("colspan", text)
        stable = "\n".join(stability_lines(rows))
        self.assertIn("lead_vehicle_brake_collision | 固定 TTC | 2 | 是", stable)
        self.assertIn("lead_vehicle_brake_collision | 状态机 | 2 | 否", stable)
        note = investigation_note(rows)
        self.assertIn("不改指标", note)
        self.assertIn("lead_vehicle_brake_collision", note)
        with tempfile.TemporaryDirectory() as folder:
            report = build_report(rows, folder, "对照", "测试集。本轮只出对照，没有用这些结果改阈值或指标。")
            page = report.read_text(encoding="utf-8")
            self.assertIn("data:image/png;base64,", page)
            self.assertNotIn("colspan", page)
            self.assertNotIn("rowspan", page)
            self.assertNotIn('src="http', page)
            self.assertNotIn('src="file', page)
            self.assertTrue((Path(folder) / "episodes.csv").is_file())
            self.assertTrue((Path(folder) / "comparison.md").is_file())

    def test_metric_modules_do_not_import_carla(self) -> None:
        for path in (ROOT / "metrics").rglob("*.py"):
            self.assertNotIn("import carla", path.read_text(encoding="utf-8"), path)


def _write_run(path: Path, *, hard_negative: bool, collided: bool, decel: float, state: str) -> Path:
    path.mkdir(parents=True)
    frames = []
    for index, (ttc, requested, accel) in enumerate(
        (
            (None, 0.0, 0.0),
            (2.0, decel, 0.0),
            (1.5, decel, -decel if decel else 0.0),
        )
    ):
        frames.append(
            {
                "frame": index,
                "sim_time_s": index * 0.05,
                "aeb_state": "IDLE" if index == 0 or decel == 0.0 else state,
                "threat": {"ttc_s": ttc},
                "control_request": {"brake": 0.0 if decel == 0.0 else 0.75, "target_decel_mps2": requested},
                "ego": {"a_long_mps2": accel},
            }
        )
    events = [
        {
            "event_type": "scenario_spec",
            "sim_time_s": 0.0,
            "detail": {
                "seed": 7,
                "ego": {"s_m": 8.0},
                "target": {"gap_m": None, "crossing_s_m": 50.0},
                "trigger": {"distance_m": 27.0},
                "end": {"hard_negative": hard_negative},
            },
        }
    ]
    if decel > 0.0:
        events.append(
            {
                "event_type": "aeb_state",
                "sim_time_s": 0.05,
                "detail": {"from": "IDLE", "to": state, "reason": "ttc_below_full"},
            }
        )
    if collided:
        events.append(
            {
                "event_type": "collision",
                "sim_time_s": 0.1,
                "detail": {"relative_speed_mps": 4.0},
            }
        )
    summary = {
        "scene_id": "synthetic",
        "strategy_name": "fixed_ttc",
        "collided": collided,
        "min_distance_m": 0.0 if collided else 3.0,
        "peak_decel_mps2": decel,
        "outcome": "collision" if collided else "avoided",
        "brake_delay_frames": 1 if decel else None,
    }
    (path / "frames.jsonl").write_text("".join(json.dumps(frame, ensure_ascii=False) + "\n" for frame in frames), encoding="utf-8")
    (path / "events.jsonl").write_text("".join(json.dumps(event, ensure_ascii=False) + "\n" for event in events), encoding="utf-8")
    (path / "summary.json").write_text(json.dumps(summary), encoding="utf-8")
    return path


def _row(scene, strategy, repeat, *, collided, hard_negative=False, speed=None, delay=2):
    return {
        "scene_id": scene,
        "strategy_name": strategy,
        "repeat_index": repeat,
        "seed": 7,
        "hard_negative": hard_negative,
        "collided": collided,
        "collision_relative_speed_mps": speed,
        "min_distance_m": 0.0 if collided else 4.0,
        "false_brake": False if hard_negative else None,
        "activation_count": 0 if delay is None else 1,
        "exit_count": 0,
        "state_switch_count": 0 if delay is None else 1,
        "risk_to_request_frames": delay,
        "request_to_decel_frames": None if delay is None else 1,
        "peak_decel_mps2": 0.0 if delay is None else 6.0,
        "peak_jerk_mps3": 0.0 if delay is None else 30.0,
        "trigger_distance_m": 55.0 if hard_negative else 14.0,
        "longitudinal_distance_m": 42.0 if hard_negative else 22.0,
        "outcome": "collision" if collided else "avoided",
    }
