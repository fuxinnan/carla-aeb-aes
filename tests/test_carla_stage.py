"""合成直线位姿能贴到一条有航向的车道上。不连接 CARLA。"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scenarios.geometry import make_straight_road
from sim.carla_stage import world_pose


class CarlaStagePoseTest(unittest.TestCase):
    def test_straight_road_keeps_arc_and_right_offset(self) -> None:
        road = make_straight_road(40.0, 2.0, yaw_rad=math.pi / 2.0)
        x_m, y_m, z_m, yaw = world_pose(road, 10.0, 2.0, 0.0)
        self.assertAlmostEqual(x_m, -2.0)
        self.assertAlmostEqual(y_m, 10.0)
        self.assertAlmostEqual(z_m, 0.0)
        self.assertAlmostEqual(yaw, math.pi / 2.0)

    def test_body_yaw_is_added_to_the_lane_heading(self) -> None:
        road = make_straight_road(40.0, 2.0, yaw_rad=0.0)
        _x, _y, _z, yaw = world_pose(road, 8.0, 0.0, -math.pi / 2.0)
        self.assertAlmostEqual(yaw, -math.pi / 2.0)


if __name__ == "__main__":
    unittest.main()
