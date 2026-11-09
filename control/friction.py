"""纵向减速度和横向加速度的合成不能超出附着圆。"""

from __future__ import annotations

import math


def limit_friction(
    decel_mps2: float,
    a_lat_mps2: float,
    mu: float,
    g_mps2: float,
) -> tuple[float, float, bool]:
    """两边都按同一个比例缩小。返回的减速度仍是制动为正。"""
    decel = max(0.0, float(decel_mps2))
    lateral = float(a_lat_mps2)
    cap = mu * g_mps2
    if cap <= 0.0:
        raise ValueError("附着圆半径必须大于 0")
    magnitude = math.hypot(decel, lateral)
    if magnitude <= cap or magnitude <= 1e-9:
        return decel, lateral, False
    scale = cap / magnitude
    return decel * scale, lateral * scale, True
