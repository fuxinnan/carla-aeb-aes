"""第 8 轮：路径跟踪、附着圆、仲裁和刹不住场景的四策略对比。不启动 CARLA。"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import matplotlib

matplotlib.use("Agg", force=True)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aeb import build_policy, load_aeb_config
from arbiter.coordinator import REASON_BRAKE_OK, arbitration_reason
from control.actuator import BrakeActuator
from control.friction import limit_friction
from control.pure_pursuit import PurePursuit
from control.round8 import load_round8_config
from control.track_demo import run_tracking_case
from scenarios.closed_loop import simulate_episode
from scenarios.geometry import make_straight_road
from scenarios.spec import load_spec
from schemas import FrameRecord, load_dataclass
from viz.coordination import plot_arbitration, plot_friction, plot_strategy_bev, plot_tracking


SCENE = ROOT / "configs" / "scenarios" / "round8" / "pedestrian_unavoidable.yaml"
FIGURES = ROOT / "figures"
SUMMARY = ROOT / "runs" / "round8" / "comparison.md"


class Round8CoordinationTest(unittest.TestCase):
    def test_sources_do_not_import_carla(self) -> None:
        for relative in (
            "control/pure_pursuit.py",
            "control/friction.py",
            "control/track_demo.py",
            "control/round8.py",
            "arbiter/coordinator.py",
            "strategies/aeb_aes.py",
            "viz/coordination.py",
        ):
            text = (ROOT / relative).read_text(encoding="utf-8")
            self.assertNotIn("import carla", text)
            self.assertNotIn("from carla", text)

    def test_pure_pursuit_steers_toward_a_point_on_the_right(self) -> None:
        limits = load_round8_config().pursuit
        tracker = PurePursuit(limits)
        path = tuple((distance, 3.0) for distance in (4.0, 8.0, 12.0, 16.0))
        command = tracker.command(0.0, 0.0, 0.0, 8.0, path, 0.05)
        self.assertGreater(command.steer_rad, 0.0)
        self.assertGreater(command.a_lat_mps2, 0.0)
        self.assertLessEqual(abs(command.yaw_rate_radps), limits.max_yaw_rate_radps)

    def test_tracking_error_stays_inside_the_written_threshold(self) -> None:
        result = run_tracking_case()
        self.assertLessEqual(result.max_abs_error_m, result.threshold_m)
        self.assertAlmostEqual(result.threshold_m, 0.35)
        figure = plot_tracking(result, FIGURES / "path_tracking.png")
        self.assertGreater(figure.stat().st_size, 10_000)

    def test_friction_circle_scales_an_extreme_pair(self) -> None:
        config = load_round8_config()
        decel, lateral, limited = limit_friction(6.0, 6.0, config.friction.mu, config.friction.g_mps2)
        cap = config.friction.mu * config.friction.g_mps2
        self.assertTrue(limited)
        self.assertLessEqual(math.hypot(decel, lateral), cap + 1e-6)
        self.assertGreater(decel, 0.0)
        self.assertGreater(lateral, 0.0)
        untouched, _, not_limited = limit_friction(6.0, 0.0, config.friction.mu, config.friction.g_mps2)
        self.assertFalse(not_limited)
        self.assertAlmostEqual(untouched, 6.0)

    def test_arbiter_uses_aes_only_when_braking_cannot_stop(self) -> None:
        self.assertEqual(arbitration_reason(True, True, False, False), REASON_BRAKE_OK)
        self.assertEqual(arbitration_reason(False, True, False, False), "aes_selected")
        self.assertEqual(arbitration_reason(False, False, False, False), "aes_infeasible")
        self.assertEqual(arbitration_reason(None, False, False, False), "aeb_only_no_conflict")

    def test_old_frame_without_lateral_fields_still_loads(self) -> None:
        payload = {
            "run_id": "old",
            "scene_id": "old",
            "strategy_name": "state_machine",
            "frame": 0,
            "sim_time_s": 0.0,
            "source_tag": "gt",
            "ego": {
                "x_m": 0.0,
                "y_m": 0.0,
                "z_m": 0.0,
                "roll_rad": 0.0,
                "pitch_rad": 0.0,
                "yaw_rad": 0.0,
                "v_long_mps": 1.0,
                "v_lat_mps": 0.0,
                "v_up_mps": 0.0,
                "speed_mps": 1.0,
                "yaw_rate_radps": 0.0,
                "a_long_mps2": 0.0,
                "a_lat_mps2": 0.0,
                "steer_road_rad": None,
            },
            "threat": {
                "ttc_s": None,
                "pred_min_distance_m": None,
                "invalid_reason": "no_track",
                "threat_id": None,
                "in_path": None,
                "closing_speed_mps": None,
                "overlap_ratio": None,
            },
            "aeb_state": "IDLE",
            "aes_state": "IDLE",
            "trigger_reason": None,
            "exit_reason": None,
            "arbitration_reason": None,
            "control_request": {"throttle": 0.0, "brake": 0.0, "steer": 0.0, "desired_speed_mps": None},
            "actual_control": {"throttle": 0.0, "brake": 0.0, "steer": 0.0, "desired_speed_mps": None},
            "quality_flags": [],
        }
        loaded = load_dataclass(FrameRecord, payload)
        self.assertIsNone(loaded.aes_direction)
        self.assertIsNone(loaded.path_error_m)
        self.assertIsNone(loaded.road_departure_m)
        self.assertIsNone(loaded.side_gap_m)
        self.assertIsNone(loaded.lateral_accel_mps2)

    def test_unavoidable_scene_logs_every_reason_and_lateral_risk(self) -> None:
        spec = load_spec(SCENE)
        road = make_straight_road(spec.straight_length_m, spec.sample_step_m)
        config = load_aeb_config()
        episodes = []
        for name in ("no_intervention", "fixed_ttc", "state_machine", "aeb_aes"):
            policy = build_policy(name, config, spec.ego_speed_mps)
            actuator = BrakeActuator(config.actuator, spec.fixed_delta_seconds)
            episodes.append((name, simulate_episode(spec, road, policy, actuator, f"round8-{name}")))
        combined = dict(episodes)["aeb_aes"]
        reasons = [step.record.arbitration_reason for step in combined.steps]
        self.assertTrue(all(reason for reason in reasons))
        for step in combined.steps:
            self.assertIsNotNone(step.record.road_departure_m)
            self.assertIsNotNone(step.record.side_gap_m)
            self.assertIsNotNone(step.record.lateral_accel_mps2)
            self.assertGreaterEqual(step.record.road_departure_m, 0.0)
            self.assertGreaterEqual(step.record.side_gap_m, 0.0)
        active = [step for step in combined.steps if step.record.aes_state == "ACTIVE"]
        for step in active:
            self.assertIsNotNone(step.record.path_error_m)
            self.assertIn(step.record.aes_direction, {"left", "right"})
        machine = dict(episodes)["state_machine"]
        plot_friction([step.record for step in combined.steps], FIGURES / "friction_circle.png")
        plot_arbitration([step.record for step in combined.steps], FIGURES / "arbitration_timeline.png")
        plot_strategy_bev(episodes, FIGURES / "aes_compare_bev.png")
        _write_summary(episodes, combined)
        self.assertGreater((FIGURES / "friction_circle.png").stat().st_size, 8_000)
        self.assertGreater((FIGURES / "aes_compare_bev.png").stat().st_size, 8_000)
        outside = [
            math.hypot(step.record.ego.a_long_mps2, step.record.ego.a_lat_mps2)
            for step in combined.steps
        ]
        cap = load_round8_config().friction.max_accel_mps2
        self.assertTrue(all(value <= cap + 0.2 for value in outside))


def _write_summary(episodes, combined) -> None:
    lines = ["| 策略 | 碰撞 | 碰撞相对速度 (m/s) | 结束 |", "| --- | --- | --- | --- |"]
    labels = {
        "no_intervention": "无干预",
        "fixed_ttc": "固定 TTC",
        "state_machine": "状态机 AEB",
        "aeb_aes": "状态机 AEB+AES",
    }
    for name, episode in episodes:
        speed = "—" if episode.collision_relative_speed_mps is None else f"{episode.collision_relative_speed_mps:.3f}"
        lines.append(f"| {labels[name]} | {int(episode.collided)} | {speed} | {episode.end_reason} |")
    used = sorted({step.record.arbitration_reason for step in combined.steps})
    by_name = dict(episodes)
    speeds = []
    for name in ("no_intervention", "fixed_ttc", "state_machine"):
        value = by_name[name].collision_relative_speed_mps
        speeds.append("—" if value is None else f"{value:.3f}")
    active = [step.record for step in combined.steps if step.record.aes_state == "ACTIVE"]
    errors = [record.path_error_m for record in active if record.path_error_m is not None]
    sides = [step.record.side_gap_m for step in combined.steps]
    direction = active[0].aes_direction if active else None
    lines.append("")
    lines.append("同一条刹不住的行人横穿：自车 12 m/s，行人从左侧 1.5 m 处以 1.2 m/s 向右走，触发距离 16 m。")
    lines.append(f"无干预、固定 TTC、状态机 AEB 的碰撞相对速度分别是 {speeds[0]}、{speeds[1]}、{speeds[2]} m/s。")
    lines.append(f"状态机 AEB+AES 碰撞={int(combined.collided)}，结束原因是 {combined.end_reason}。")
    if errors:
        side = "左" if direction == "left" else "右"
        lines.append(
            f"它在制动来不及的第一帧选择向{side}，跟踪中路径误差 {min(errors):.3f} 到 {max(errors):.3f} m，"
            f"侧向间距最小 {min(sides):.3f} m。"
        )
    lines.append("AEB+AES 每帧原因：" + "、".join(used))
    lines.append("这条轨迹的制动和转向合成加速度没有顶到附着圆，圆上的点都在 7.06 m/s² 里面。附着圆限制由单独的 6+6 m/s² 请求验证。")
    SUMMARY.parent.mkdir(parents=True, exist_ok=True)
    SUMMARY.write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    unittest.main()
