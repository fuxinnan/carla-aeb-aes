"""五个手算用例的示意图。图上同时写期望 TTC 和实际 TTC。"""

from __future__ import annotations

from pathlib import Path

from scenarios.geometry import box_corners, forward_xy, right_xy
from threat.assessor import REASON_TEXT
from viz.common import _prepare_matplotlib


def save_unit_case(
    path: str | Path,
    *,
    title: str,
    ego: dict,
    target: dict,
    expected_ttc_s: float | None,
    actual_ttc_s: float | None,
    expected_reason: str | None,
    actual_reason: str | None,
    conflict_xy: tuple[float, float] | None,
) -> Path:
    """画自车、目标和速度箭头。期望与实际不一致时标题仍照实写，测试会另外失败。"""
    _prepare_matplotlib()
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(7.2, 6.4))
    _box(axis, ego, "#1565C0", "自车")
    _box(axis, target, "#E53935", "目标")
    _arrow(axis, ego, "#1565C0")
    _arrow(axis, target, "#E53935")
    if conflict_xy is not None:
        axis.scatter([conflict_xy[0]], [conflict_xy[1]], marker="X", s=70, color="#212121", zorder=5, label="冲突点")
    axis.relim()
    axis.autoscale_view()
    axis.margins(0.15)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.3)
    axis.set_xlabel("世界 X (m)")
    axis.set_ylabel("世界 Y (m)")
    axis.set_title(title)
    axis.legend(loc="best", frameon=False)
    note = "\n".join(
        [
            f"期望 TTC：{_format_ttc(expected_ttc_s, expected_reason)}",
            f"实际 TTC：{_format_ttc(actual_ttc_s, actual_reason)}",
        ]
    )
    axis.text(
        0.02,
        0.98,
        note,
        transform=axis.transAxes,
        va="top",
        ha="left",
        fontsize=11,
        bbox={"boxstyle": "round", "facecolor": "white", "edgecolor": "#BDBDBD", "alpha": 0.92},
    )
    figure.tight_layout()
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=120, bbox_inches="tight")
    plt.close(figure)
    return output


def _box(axis, body: dict, color: str, label: str) -> None:
    from matplotlib.patches import Polygon

    corners = box_corners(body["x_m"], body["y_m"], body["yaw_rad"], body["length_m"], body["width_m"])
    axis.add_patch(Polygon(corners, closed=True, facecolor=color, edgecolor=color, alpha=0.35, linewidth=1.5, label=label))


def _arrow(axis, body: dict, color: str) -> None:
    fx, fy = forward_xy(body["yaw_rad"])
    rx, ry = right_xy(body["yaw_rad"])
    vx = fx * body["v_long_mps"] + rx * body.get("v_lat_mps", 0.0)
    vy = fy * body["v_long_mps"] + ry * body.get("v_lat_mps", 0.0)
    if abs(vx) + abs(vy) < 1e-6:
        return
    scale = 0.45
    axis.annotate(
        "",
        xy=(body["x_m"] + vx * scale, body["y_m"] + vy * scale),
        xytext=(body["x_m"], body["y_m"]),
        arrowprops={"arrowstyle": "->", "color": color, "lw": 1.4},
    )


def _format_ttc(ttc_s: float | None, reason: str | None) -> str:
    if ttc_s is None:
        text = REASON_TEXT.get(reason or "", reason or "")
        return f"无（{text}）" if text else "无"
    return f"{float(ttc_s):.2f} s"
