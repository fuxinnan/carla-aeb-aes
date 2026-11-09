"""重复运行的触发帧和碰撞相对速度。横轴是运行序号。"""

from __future__ import annotations

import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if not any(Path(entry).resolve() == _PROJECT_ROOT for entry in sys.path if entry):
    sys.path.insert(0, str(_PROJECT_ROOT))

from viz.common import _prepare_matplotlib


def save_stability_plot(output: str | Path, rows: list[dict], *, title: str) -> Path:
    """散点图。点落在同一条水平线上，说明这组重复没有波动。"""
    _prepare_matplotlib()
    import matplotlib.pyplot as plt

    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    indexes = [int(row["index"]) for row in rows]
    triggers = [None if row["trigger_frame"] is None else int(row["trigger_frame"]) for row in rows]
    speeds = [None if row["collision_relative_speed_mps"] is None else float(row["collision_relative_speed_mps"]) for row in rows]
    figure, axes = plt.subplots(1, 2, figsize=(10, 4))
    speed_axis, trigger_axis = axes
    _scatter(speed_axis, indexes, speeds, "#E53935")
    speed_axis.set_title("碰撞相对速度")
    speed_axis.set_ylabel("相对速度 (m/s)")
    _scatter(trigger_axis, indexes, triggers, "#1E88E5")
    trigger_axis.set_title("触发帧")
    trigger_axis.set_ylabel("帧号")
    for axis in axes:
        axis.set_xlabel("运行序号")
        axis.grid(True, alpha=0.3)
        axis.set_xticks(indexes)
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(path, dpi=120)
    plt.close(figure)
    return path


def _scatter(axis, indexes: list[int], values: list[float | None], color: str) -> None:
    xs = [index for index, value in zip(indexes, values) if value is not None]
    ys = [float(value) for value in values if value is not None]
    axis.scatter(xs, ys, color=color, s=36, zorder=3)
    if len(ys) >= 1:
        low = min(ys)
        high = max(ys)
        if high - low < 1e-6:
            pad = 1.0 if high == 0.0 else max(1.0, abs(high) * 0.05)
            axis.set_ylim(low - pad, high + pad)
