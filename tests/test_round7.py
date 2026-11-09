"""第 7 轮：AES 候选轨迹和可行性。不启动 CARLA，不改 AEB。"""

from __future__ import annotations

import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg", force=True)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aes.cases import build_cases
from aes.config import load_aes_config
from aes.planner import plan_avoidance
from aes.types import (
    REASON_LAT_ACCEL,
    REASON_OVERLAP,
    REASON_ROAD,
    REASON_STEER,
    REASON_TEXT,
    REASON_TIE,
    REASON_TOO_CLOSE,
    DrivableCorridor,
)
from viz.aes_plan import render_aes_figures


FIGURE_DIR = ROOT / "figures"


class Round7AesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = load_aes_config()
        cls.cases = {case.case_id: case for case in build_cases()}
        cls.plans = {
            case_id: plan_avoidance(case.ego, case.targets, case.road, case.corridor, cls.config)
            for case_id, case in cls.cases.items()
        }

    def test_sources_do_not_import_carla_or_aeb(self) -> None:
        for relative in (
            "aes/config.py",
            "aes/types.py",
            "aes/trajectory.py",
            "aes/check.py",
            "aes/planner.py",
            "aes/cases.py",
            "viz/aes_plan.py",
        ):
            text = (ROOT / relative).read_text(encoding="utf-8")
            self.assertNotIn("import carla", text)
            self.assertNotIn("from carla", text)
            self.assertNotIn("import aeb", text)
            self.assertNotIn("from aeb", text)

    def test_four_cases_select_the_expected_side(self) -> None:
        left = self.plans["left_clear"]
        right = self.plans["right_clear"]
        blocked = self.plans["both_blocked"]
        close = self.plans["too_close"]

        self.assertTrue(left.avoidable)
        self.assertEqual(left.direction, "left")
        self.assertEqual(left.rejections[0].direction, "right")
        self.assertEqual(left.rejections[0].reasons, (REASON_OVERLAP,))
        self.assertGreater(left.candidates[0].min_target_gap_m, 0.5)
        self.assertGreater(left.candidates[1].first_overlap_time_s, 1.0)

        self.assertTrue(right.avoidable)
        self.assertEqual(right.direction, "right")
        self.assertEqual(right.rejections[0].reasons, (REASON_OVERLAP,))
        self.assertGreater(right.candidates[1].min_target_gap_m, 0.5)

        self.assertFalse(blocked.avoidable)
        self.assertIsNone(blocked.direction)
        self.assertEqual(blocked.points, ())
        self.assertEqual(blocked.cost, None)
        self.assertEqual([item.reasons for item in blocked.rejections], [(REASON_OVERLAP,), (REASON_OVERLAP,)])

        self.assertFalse(close.avoidable)
        self.assertIsNone(close.direction)
        self.assertEqual([item.reasons for item in close.rejections], [(REASON_TOO_CLOSE,), (REASON_TOO_CLOSE,)])
        self.assertLess(close.candidates[0].first_overlap_time_s, 0.8)

    def test_rejection_text_is_readable(self) -> None:
        for plan in self.plans.values():
            self.assertGreaterEqual(len(plan.rejections), 1)
            for rejection in plan.rejections:
                self.assertTrue(rejection.reasons)
                self.assertTrue(rejection.text.strip())
                for reason in rejection.reasons:
                    self.assertIn(REASON_TEXT[reason], rejection.text)
            self.assertIn("1.75 m", plan.boundary_source)
            self.assertIn("-5.25 m", plan.boundary_source)

    def test_both_feasible_keeps_left_and_rejects_right(self) -> None:
        case = self.cases["left_clear"]
        plan = plan_avoidance(case.ego, (), case.road, case.corridor, self.config)
        self.assertTrue(plan.avoidable)
        self.assertEqual(plan.direction, "left")
        self.assertTrue(all(candidate.feasible for candidate in plan.candidates))
        self.assertEqual(plan.rejections[0].direction, "right")
        self.assertEqual(plan.rejections[0].reasons, (REASON_TIE,))
        self.assertIn("保留", plan.rejections[0].text)

    def test_body_not_centerline_is_checked_against_the_boundary(self) -> None:
        case = self.cases["left_clear"]
        narrow = DrivableCorridor(case.corridor.left_m, 4.0, case.corridor.source)
        plan = plan_avoidance(case.ego, (), case.road, narrow, self.config)
        right = plan.candidates[1]
        left = plan.candidates[0]
        self.assertFalse(right.feasible)
        self.assertIn(REASON_ROAD, right.reasons)
        self.assertTrue(left.feasible)
        self.assertEqual(plan.direction, "left")

    def test_short_trajectory_exceeds_the_lateral_limit(self) -> None:
        case = self.cases["left_clear"]
        short = replace(self.config, trajectory=replace(self.config.trajectory, length_m=10.0))
        plan = plan_avoidance(case.ego, (), case.road, case.corridor, short)
        self.assertFalse(plan.avoidable)
        self.assertIsNone(plan.direction)
        for candidate in plan.candidates:
            self.assertIn(REASON_LAT_ACCEL, candidate.reasons)
            self.assertGreater(candidate.peak_lat_accel_mps2, self.config.constraints.max_lat_accel_mps2)

    def test_reference_maneuver_stays_inside_limits_and_keeps_an_s_shape(self) -> None:
        case = self.cases["left_clear"]
        plan = plan_avoidance(case.ego, (), case.road, case.corridor, self.config)
        limits = self.config.constraints
        for candidate in plan.candidates:
            self.assertLess(candidate.peak_lat_accel_mps2, limits.max_lat_accel_mps2)
            self.assertGreater(candidate.peak_lat_accel_mps2, 1.5)
            self.assertLess(candidate.peak_steer_rad, limits.max_steer_rad)
            self.assertLess(candidate.peak_steer_rate_radps, limits.max_steer_rate_radps)
            y_values = [point.y_m for point in candidate.points]
            if candidate.direction == "right":
                self.assertEqual(y_values, sorted(y_values))
                self.assertGreater(candidate.points[2].a_lat_mps2, 0.0)
            else:
                self.assertEqual(y_values, sorted(y_values, reverse=True))
                self.assertLess(candidate.points[2].a_lat_mps2, 0.0)
            self.assertAlmostEqual(candidate.points[0].x_m, case.ego.x_m, delta=1e-6)
            self.assertAlmostEqual(candidate.points[0].y_m, 0.0, delta=1e-6)
            self.assertAlmostEqual(abs(candidate.points[-1].y_m), self.config.trajectory.lateral_offset_m, delta=1e-6)
            self.assertLess(abs(candidate.points[-1].yaw_rad), 1e-6)
            self.assertLess(abs(candidate.points[-1].steer_rad), 1e-6)
            self.assertLess(max(abs(point.yaw_rad) for point in candidate.points), 0.35)

    def test_very_short_path_also_exceeds_steer_angle(self) -> None:
        case = self.cases["left_clear"]
        sharp = replace(self.config, trajectory=replace(self.config.trajectory, length_m=8.0))
        plan = plan_avoidance(case.ego, (), case.road, case.corridor, sharp)
        self.assertIn(REASON_STEER, plan.candidates[0].reasons)
        self.assertGreater(plan.candidates[0].peak_steer_rad, self.config.constraints.max_steer_rad)

    def test_unknown_config_key_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            load_aes_config(ROOT / "configs" / "aeb" / "round5.yaml")

    def test_figures_show_cases_and_constraint_lines(self) -> None:
        overview, dynamics = render_aes_figures(FIGURE_DIR, self.config)
        self.assertEqual(overview.name, "aes_cases.png")
        self.assertGreater(overview.stat().st_size, 20_000)
        self.assertGreater(dynamics.stat().st_size, 20_000)
        self.assertTrue(math.isfinite(self.plans["left_clear"].cost))


if __name__ == "__main__":
    unittest.main()
