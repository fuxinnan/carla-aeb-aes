"""第 3 轮：模板校验、无干预碰撞，以及 20 次运动学重复完全一致。不启动 CARLA。"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import matplotlib

matplotlib.use("Agg", force=True)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scenarios.geometry import make_straight_road
from scenarios.motion import rollout
from scenarios.report import assess_repeats
from scenarios.spec import load_spec
from viz.replay_bev import export_bev_replay
from viz.scenario_preview import save_scenario_preview
from viz.stability import save_stability_plot


PED_YAML = ROOT / "configs" / "scenarios" / "pedestrian_crossing_collision.yaml"
LEAD_YAML = ROOT / "configs" / "scenarios" / "lead_vehicle_brake_collision.yaml"


class Round3ScenarioTest(unittest.TestCase):
    def test_examples_keep_template_behavior_and_instance_numbers(self) -> None:
        pedestrian = load_spec(PED_YAML)
        lead = load_spec(LEAD_YAML)
        self.assertEqual(pedestrian.template, "pedestrian_crossing")
        self.assertEqual(pedestrian.instance, "collision")
        self.assertEqual(pedestrian.target_behavior, "fixed_speed_walk")
        self.assertEqual(pedestrian.ego_behavior, "hold_initial_speed")
        self.assertEqual(pedestrian.ego_speed_mps, 10.0)
        self.assertEqual(pedestrian.target_speed_mps, 1.5)
        self.assertEqual(pedestrian.trigger_distance_m, 27.0)
        self.assertFalse(pedestrian.hard_negative)
        self.assertEqual(lead.target_behavior, "fixed_deceleration")
        self.assertEqual(lead.target_decel_mps2, 4.0)
        self.assertEqual(lead.target_gap_m, 22.0)
        self.assertEqual(pedestrian.seed, lead.seed)
        self.assertNotIn("traffic_manager", pedestrian.target_behavior)

    def test_instance_cannot_change_behavior_or_drop_required_numbers(self) -> None:
        with tempfile.TemporaryDirectory() as temp_name:
            folder = Path(temp_name)
            managed = folder / "managed.yaml"
            managed.write_text(
                "template: pedestrian_crossing\ninstance: bad\nseed: 1\nbehavior:\n  target: traffic_manager\n",
                encoding="utf-8",
            )
            with self.assertRaises(ValueError):
                load_spec(managed)
            missing = folder / "missing.yaml"
            missing.write_text(
                "template: pedestrian_crossing\ninstance: bad\nseed: 1\nego:\n  speed_mps: 10\n",
                encoding="utf-8",
            )
            with self.assertRaises(ValueError):
                load_spec(missing)
            extra = folder / "extra.yaml"
            extra.write_text(
                "\n".join(
                    [
                        "template: lead_vehicle_brake",
                        "instance: bad",
                        "seed: 1",
                        "ego:",
                        "  speed_mps: 11",
                        "target:",
                        "  speed_mps: 6",
                        "  decel_mps2: 4",
                        "  gap_m: 22",
                        "  lateral_m: 3",
                        "trigger:",
                        "  distance_m: 14",
                        "end:",
                        "  hard_negative: false",
                    ]
                ),
                encoding="utf-8",
            )
            with self.assertRaises(ValueError):
                load_spec(extra)

    def test_spawn_functions_need_a_bound_runtime(self) -> None:
        from scenarios.spawn import lead_vehicle_brake, pedestrian_crossing

        spec = load_spec(PED_YAML)
        with self.assertRaises(RuntimeError):
            pedestrian_crossing(spec)
        with self.assertRaises(RuntimeError):
            lead_vehicle_brake(load_spec(LEAD_YAML))

    def test_both_examples_collide_and_twenty_rollouts_match(self) -> None:
        road = make_straight_road(length_m=120.0, step_m=1.0, yaw_rad=0.2)
        for path in (PED_YAML, LEAD_YAML):
            spec = load_spec(path)
            runs = [rollout(spec, road) for _ in range(20)]
            first = runs[0]
            self.assertIsNotNone(first.trigger_frame)
            self.assertIsNotNone(first.collision_frame)
            self.assertLess(first.trigger_frame, first.collision_frame)
            self.assertEqual(first.end_reason, "collision")
            self.assertGreater(first.collision_relative_speed_mps, 1.0)
            speeds = [item.collision_relative_speed_mps for item in runs]
            triggers = [item.trigger_frame for item in runs]
            collided = [item.collision_frame is not None for item in runs]
            spawns = [(item.steps[0].target.x_m, item.steps[0].target.y_m, item.steps[0].target.z_m) for item in runs]
            self.assertEqual(len(set(speeds)), 1, speeds)
            self.assertEqual(len(set(triggers)), 1, triggers)
            self.assertEqual(set(collided), {True})
            self.assertEqual(len(set(spawns)), 1, spawns)
            report = assess_repeats([_result(index, item) for index, item in enumerate(runs)])
            self.assertTrue(report["stable"], report)
            self.assertEqual(report["speed_range_mps"], 0.0)

    def test_preview_stability_and_replay_mark_trigger(self) -> None:
        spec = load_spec(PED_YAML)
        road = make_straight_road()
        plan = rollout(spec, road)
        from scenarios.geometry import lane_boundaries

        left, right = lane_boundaries(road)
        trigger = next(step for step in plan.steps if step.trigger_edge)
        collision = plan.steps[plan.collision_frame]
        with tempfile.TemporaryDirectory() as temp_name:
            folder = Path(temp_name)
            preview = save_scenario_preview(
                folder / "scenario_preview.png",
                spec_fields=spec.key_fields(),
                ego_xy=[(step.ego.x_m, step.ego.y_m) for step in plan.steps],
                target_xy=[(step.target.x_m, step.target.y_m) for step in plan.steps],
                left_xy=left,
                right_xy=right,
                trigger_xy=(trigger.ego.x_m, trigger.ego.y_m),
                collision_xy=(
                    (collision.ego.x_m + collision.target.x_m) * 0.5,
                    (collision.ego.y_m + collision.target.y_m) * 0.5,
                ),
            )
            rows = [
                {
                    "index": index,
                    "trigger_frame": plan.trigger_frame,
                    "collision_relative_speed_mps": plan.collision_relative_speed_mps,
                    "collided": True,
                }
                for index in range(20)
            ]
            stability = save_stability_plot(folder / "stability.png", rows, title=spec.scene_id)
            run_dir = folder / "rep_00"
            run_dir.mkdir()
            frames = []
            targets = []
            for step in plan.steps[:8]:
                frames.append(
                    '{"scene_id":"pedestrian_crossing_collision","sim_time_s":%s,"ego":{"x_m":%s,"y_m":%s}}\n'
                    % (step.sim_time_s, step.ego.x_m, step.ego.y_m)
                )
                targets.append(
                    '{"frame":%s,"sim_time_s":%s,"track":{"track_id":"ped_01","x_m":%s,"y_m":%s}}\n'
                    % (step.frame, step.sim_time_s, step.target.x_m, step.target.y_m)
                )
            (run_dir / "frames.jsonl").write_text("".join(frames), encoding="utf-8")
            (run_dir / "targets.jsonl").write_text("".join(targets), encoding="utf-8")
            (run_dir / "events.jsonl").write_text(
                '{"event_type":"target_trigger","sim_time_s":%s,"detail":{"frame":%s}}\n'
                % (trigger.sim_time_s, trigger.frame),
                encoding="utf-8",
            )
            replay = export_bev_replay(run_dir, env_path=ROOT / "env.json")
            for path in (preview, stability, replay):
                self.assertTrue(path.is_file(), path)
                self.assertGreater(path.stat().st_size, 500)


def _result(index: int, plan) -> dict:
    step = plan.steps[0].target
    ego = plan.steps[0].ego
    return {
        "index": index,
        "trigger_frame": plan.trigger_frame,
        "collided": plan.collision_frame is not None,
        "collision_relative_speed_mps": plan.collision_relative_speed_mps,
        "target_spawn": {"x_m": step.x_m, "y_m": step.y_m, "z_m": step.z_m, "yaw_rad": step.yaw_rad},
        "ego_spawn": {"x_m": ego.x_m, "y_m": ego.y_m, "z_m": ego.z_m, "yaw_rad": ego.yaw_rad},
    }


if __name__ == "__main__":
    unittest.main()
