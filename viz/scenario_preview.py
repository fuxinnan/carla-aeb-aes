"""场景俯视预览。只画已经算好的路径和车道边界，不连接仿真器。"""

from __future__ import annotations

import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if not any(Path(entry).resolve() == _PROJECT_ROOT for entry in sys.path if entry):
    sys.path.insert(0, str(_PROJECT_ROOT))

from viz.common import _prepare_matplotlib


def save_scenario_preview(
    output: str | Path,
    *,
    spec_fields: dict,
    ego_xy: list[tuple[float, float]],
    target_xy: list[tuple[float, float]],
    left_xy: list[tuple[float, float]],
    right_xy: list[tuple[float, float]],
    trigger_xy: tuple[float, float] | None,
    collision_xy: tuple[float, float] | None,
) -> Path:
    """俯视图：车道边界、自车路径、目标路径、触发点和预碰撞点。"""
    _prepare_matplotlib()
    import matplotlib.pyplot as plt

    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(8, 8))
    if left_xy:
        axis.plot([point[0] for point in left_xy], [point[1] for point in left_xy], color="#90A4AE", linewidth=1.2, label="车道边界")
    if right_xy:
        axis.plot([point[0] for point in right_xy], [point[1] for point in right_xy], color="#90A4AE", linewidth=1.2)
    if ego_xy:
        axis.plot([point[0] for point in ego_xy], [point[1] for point in ego_xy], color="#1E88E5", linewidth=2.0, label="自车路径")
        axis.scatter([ego_xy[0][0]], [ego_xy[0][1]], color="#1E88E5", s=28, zorder=3)
    if target_xy:
        axis.plot([point[0] for point in target_xy], [point[1] for point in target_xy], color="#FB8C00", linewidth=2.0, label="目标路径")
        axis.scatter([target_xy[0][0]], [target_xy[0][1]], color="#FB8C00", s=28, zorder=3)
    if trigger_xy is not None:
        axis.scatter([trigger_xy[0]], [trigger_xy[1]], marker="^", color="#43A047", s=70, zorder=4, label="触发点")
    if collision_xy is not None:
        axis.scatter([collision_xy[0]], [collision_xy[1]], marker="X", color="#E53935", s=80, zorder=4, label="预碰撞点")
    axis.set_aspect("equal", adjustable="datalim")
    axis.set_xlabel("世界 X (m)")
    axis.set_ylabel("世界 Y (m)")
    axis.grid(True, alpha=0.3)
    axis.legend(frameon=False, loc="upper right")
    axis.text(
        0.02,
        0.02,
        _caption(spec_fields),
        transform=axis.transAxes,
        va="bottom",
        ha="left",
        fontsize=9,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9, "edgecolor": "#CFD8DC"},
    )
    figure.tight_layout()
    figure.savefig(path, dpi=120)
    plt.close(figure)
    return path


def _caption(fields: dict) -> str:
    ego = fields.get("ego") or {}
    target = fields.get("target") or {}
    trigger = fields.get("trigger") or {}
    end = fields.get("end") or {}
    decel = target.get("decel_mps2")
    decel_text = "" if decel is None else f"\n目标减速度 {float(decel):.1f} m/s²"
    return (
        f"模板 {fields.get('template')} / 实例 {fields.get('instance')}\n"
        f"种子 {fields.get('seed')}\n"
        f"自车初速 {float(ego.get('speed_mps', 0.0)):.1f} m/s\n"
        f"目标速度 {float(target.get('speed_mps', 0.0)):.1f} m/s"
        f"{decel_text}\n"
        f"触发距离 {float(trigger.get('distance_m', 0.0)):.1f} m\n"
        f"硬负例 {'是' if end.get('hard_negative') else '否'}"
    )
