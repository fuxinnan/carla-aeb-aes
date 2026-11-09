"""第 10 轮：测试集不能写，邻域场景沿用第 9 轮加载器，晚制动线索能回放。"""

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

from round10.guard import refuse_test_write
from round10.mine import _cause, _rules, _timing
from round10.neighborhood import write_neighborhood
from round10.replay import replay
from round9.catalog import load_split


class Round10Test(unittest.TestCase):
    def test_sources_do_not_import_carla(self) -> None:
        for relative in (
            "round10/guard.py",
            "round10/mine.py",
            "round10/replay.py",
            "round10/neighborhood.py",
            "round10/regress.py",
            "round10/run.py",
            "viz/round10.py",
        ):
            text = (ROOT / relative).read_text(encoding="utf-8")
            self.assertNotIn("import carla", text)
            self.assertNotIn("from carla", text)

    def test_writing_into_the_test_set_is_rejected(self) -> None:
        for relative in (
            "configs/scenarios/test/new_case.yaml",
            "configs/scenarios/round9/test/new_case.yaml",
        ):
            with self.assertRaises(PermissionError):
                refuse_test_write(ROOT / relative)

    def test_late_brake_needs_a_request_after_the_avoidable_frame(self) -> None:
        frames = [
            _frame(0, ttc=1.2, avoidable=True, decel=0.0),
            _frame(1, ttc=0.9, avoidable=True, decel=0.0),
            _frame(2, ttc=0.6, avoidable=False, decel=3.5),
        ]
        timing = _timing(frames)
        evidence = {
            "collided": True,
            "min_distance_m": 0.0,
            "state_switch_count": 1,
            "false_brake": None,
            "request_to_decel_frames": 1,
            "max_abs_path_error_m": None,
            "aes_infeasible_frames": [],
            "gap_frames": [],
            "first_risk_frame": timing["first_risk_frame"],
            "avoidable_critical_frame": timing["critical_frame"],
            "brake_request_frame": timing["request_frame"],
            "risk_before_or_at_critical": timing["risk_known_in_time"],
        }
        thresholds = {"slow_response_frames": 3, "path_error_m": 0.35}
        self.assertEqual(_cause(evidence, thresholds), "late_brake")
        self.assertIn("collision", _rules(evidence, {"near_miss_m": 0.5, "state_switch_count": 12, "path_error_m": 0.35}))

    def test_neighborhood_yaml_loads_as_tuning(self) -> None:
        case = {
            "scene_id": "left_avoidable_v11_d16",
            "ego_speed_mps": 11.0,
            "distance_m": 16.0,
            "appearance_s": 0.0,
            "manifest": {"template": "left_avoidable"},
        }
        with tempfile.TemporaryDirectory() as temp_name:
            directory = Path(temp_name) / "tuning"
            written = write_neighborhood([case], directory)
            self.assertTrue(written)
            loaded = load_split(directory, "tuning")
            self.assertTrue(all(scene.split == "tuning" for scene in loaded))
            self.assertTrue(any(scene.ego_speed_mps == 12.0 and scene.distance_m == 18.0 for scene in loaded))

    def test_replay_of_a_logged_late_brake_keeps_the_clue(self) -> None:
        manifest = {
            "scene_id": "left_avoidable_v11_d16",
            "strategy_name": "state_machine",
            "split": "tuning",
            "seed": 22,
            "template": "left_avoidable",
            "instance": "v11_d16",
            "ego_speed_mps": 11.0,
            "distance_m": 16.0,
            "hard_negative": False,
            "occlusion": False,
            "aeb_config": "configs/aeb/round9/search/state_c3_p1_8.yaml",
        }
        result = replay(manifest)
        frames = [
            {
                "frame": record.frame,
                "threat": {"ttc_s": record.threat.ttc_s, "brake_avoidable": record.threat.brake_avoidable},
                "control_request": {"target_decel_mps2": record.control_request.target_decel_mps2},
                "path_error_m": record.path_error_m,
                "arbitration_reason": record.arbitration_reason,
            }
            for record in result.records
        ]
        timing = _timing(frames)
        evidence = {
            "collided": result.collided,
            "false_brake": None,
            "request_to_decel_frames": 1,
            "max_abs_path_error_m": None,
            "aes_infeasible_frames": [],
            "gap_frames": [],
            "first_risk_frame": timing["first_risk_frame"],
            "avoidable_critical_frame": timing["critical_frame"],
            "brake_request_frame": timing["request_frame"],
            "risk_before_or_at_critical": timing["risk_known_in_time"],
        }
        self.assertTrue(result.collided)
        self.assertEqual(_cause(evidence, {"slow_response_frames": 3, "path_error_m": 0.35}), "late_brake")
        self.assertLessEqual(timing["first_risk_frame"], timing["critical_frame"])
        self.assertGreater(timing["request_frame"], timing["critical_frame"])


def _frame(number, ttc, avoidable, decel):
    return {
        "frame": number,
        "threat": {"ttc_s": ttc, "brake_avoidable": avoidable},
        "control_request": {"target_decel_mps2": decel},
        "path_error_m": None,
        "arbitration_reason": None,
    }


if __name__ == "__main__":
    unittest.main()
