"""第 9 轮：遮挡会推迟看见行人，调参记录里没有测试集。"""

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
from round9.catalog import write_grid
from round9.simulate import DT_S, simulate_scene
from round9.study import HYSTERESIS_GAP_S, Journal, ablation_configs, write_aeb_yaml
from schemas import Track, load_dataclass
from round9.catalog import GridScene


class Round9Test(unittest.TestCase):
    def test_sources_do_not_import_carla(self) -> None:
        for relative in (
            "perception/visibility.py",
            "threat/pipeline.py",
            "viz/round9.py",
            "round9/catalog.py",
            "round9/simulate.py",
            "round9/study.py",
            "round9/run.py",
        ):
            text = (ROOT / relative).read_text(encoding="utf-8")
            self.assertNotIn("import carla", text)
            self.assertNotIn("from carla", text)

    def test_old_track_without_first_seen_loads_none(self) -> None:
        loaded = load_dataclass(
            Track,
            {
                "track_id": "a",
                "source": "gt",
                "class_name": "vehicle",
                "x_m": 1.0,
                "y_m": 2.0,
                "z_m": 0.0,
                "yaw_rad": 0.0,
                "v_long_mps": 0.0,
                "v_lat_mps": 0.0,
                "speed_mps": 0.0,
                "length_m": 4.0,
                "width_m": 2.0,
                "height_m": 1.5,
                "rel_x_m": 1.0,
                "rel_y_m": 2.0,
                "rel_v_long_mps": 0.0,
                "rel_v_lat_mps": 0.0,
                "in_path": False,
            },
        )
        self.assertIsNone(loaded.first_seen_s)
        self.assertIsNone(loaded.timestamp_s)

    def test_occlusion_delays_the_hidden_pedestrian(self) -> None:
        scene = GridScene("occlusion_reveal", "probe", 1, "tuning", 10.0, 22.0, False)
        config = load_aeb_config(ROOT / "configs" / "aeb" / "round5.yaml")
        full = _run(scene, config, False)
        hidden = _run(scene, config, True)
        full_frame = _risk(full, "ped_hidden")
        hidden_frame = _risk(hidden, "ped_hidden")
        self.assertIsNotNone(full_frame)
        self.assertIsNotNone(hidden_frame)
        self.assertGreater(hidden_frame, full_frame)
        self.assertGreater(hidden.first_seen["ped_hidden"], full_frame * DT_S)
        self.assertNotIn("ped_hidden", {item for item in hidden.first_seen if hidden.first_seen[item] <= 0.0})

    def test_parallel_vehicle_is_a_hard_negative(self) -> None:
        scene = GridScene("parallel_vehicle", "probe", 2, "tuning", 10.0, 18.0, True)
        config = load_aeb_config(ROOT / "configs" / "aeb" / "round5.yaml")
        result = _run(scene, config, False)
        self.assertFalse(result.collided)
        self.assertIsNone(result.first_ttc_frame)
        self.assertTrue(all(record.control_request.target_decel_mps2 == 0.0 for record in result.records))

    def test_tuning_journal_does_not_open_the_test_split(self) -> None:
        with tempfile.TemporaryDirectory() as temp_name:
            root = Path(temp_name)
            write_grid(root)
            journal = Journal()
            scenes = journal.load_tuning(root)
            self.assertTrue(scenes)
            self.assertTrue(all(scene.split == "tuning" for scene in scenes))
            snapshot = journal.snapshot()
            self.assertFalse(snapshot["test_dir_opened"])
            self.assertTrue(all(Path(path).parent.name == "tuning" for path in snapshot["opened_paths"]))
            self.assertTrue((root / "test").is_dir())

    def test_ablation_only_changes_one_configuration_item(self) -> None:
        base = load_aeb_config(ROOT / "configs" / "aeb" / "round5.yaml")
        with tempfile.TemporaryDirectory() as temp_name:
            chosen = write_aeb_yaml(Path(temp_name) / "chosen.yaml", base)
            rows = {token: config for token, _label, config in ablation_configs(chosen)}
            hysteresis = rows["no_hysteresis"].state_machine
            self.assertAlmostEqual(hysteresis.ttc_warn_off_s - hysteresis.ttc_warn_on_s, HYSTERESIS_GAP_S)
            self.assertAlmostEqual(hysteresis.ttc_partial_off_s - hysteresis.ttc_partial_on_s, HYSTERESIS_GAP_S)
            self.assertAlmostEqual(hysteresis.ttc_full_off_s - hysteresis.ttc_full_on_s, HYSTERESIS_GAP_S)
            self.assertEqual(rows["no_confirm"].state_machine.confirm_frames, 1)
            self.assertEqual(rows["no_grading"].state_machine.partial_decel_mps2, rows["no_grading"].state_machine.full_decel_mps2)
            self.assertEqual(rows["full"].state_machine.confirm_frames, chosen.state_machine.confirm_frames)
            loaded = load_aeb_config(write_aeb_yaml(Path(temp_name) / "tight.yaml", rows["no_hysteresis"]).path)
            self.assertAlmostEqual(loaded.state_machine.ttc_partial_off_s - loaded.state_machine.ttc_partial_on_s, HYSTERESIS_GAP_S)


def _run(scene, config, occlusion):
    policy = build_policy("state_machine", config, scene.ego_speed_mps)
    return simulate_scene(scene, policy, BrakeActuator(config.actuator, DT_S), occlusion=occlusion)


def _risk(result, track_id: str) -> int | None:
    for record in result.records:
        if record.threat.threat_id == track_id and record.threat.ttc_s is not None:
            return record.frame
    return None


if __name__ == "__main__":
    unittest.main()
