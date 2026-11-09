"""对照图只吃指标表。颜色按策略名固定。"""

from __future__ import annotations

from pathlib import Path

from metrics.aggregate import LABELS, STRATEGIES, sample_std, strategy_repeat_values

COLORS = {
    "no_intervention": "#546E7A",
    "fixed_ttc": "#C62828",
    "state_machine": "#1565C0",
}


def plot_overview(rows: list[dict], path: str | Path) -> Path:
    """碰撞率、碰撞相对速度和硬负例误制动率。误差线来自重复之间的差别。"""
    _prepare()
    import matplotlib.pyplot as plt
    import numpy as np

    figure, axes = plt.subplots(1, 3, figsize=(11, 3.6))
    specs = (
        (axes[0], "rate_collided", "碰撞率"),
        (axes[1], "collision_speed", "碰撞相对速度均值 (m/s)"),
        (axes[2], "false_brake", "硬负例误制动率"),
    )
    positions = np.arange(len(STRATEGIES))
    for axis, kind, title in specs:
        heights = []
        errors = []
        for name in STRATEGIES:
            values = strategy_repeat_values(rows, name, kind)
            if not values:
                heights.append(np.nan)
                errors.append(0.0)
            else:
                heights.append(sum(values) / len(values))
                errors.append(sample_std(values))
        axis.bar(
            positions,
            heights,
            yerr=errors,
            color=[COLORS[name] for name in STRATEGIES],
            capsize=4,
            width=0.7,
        )
        axis.set_xticks(positions, [LABELS[name] for name in STRATEGIES])
        axis.set_title(title)
        axis.set_ylim(bottom=0.0)
        axis.grid(True, axis="y", alpha=0.3)
    figure.tight_layout()
    return _save(figure, path)


def plot_delay_boxes(rows: list[dict], path: str | Path) -> Path:
    """两种延迟的分布。没有请求的运行不进入箱线。"""
    _prepare()
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 2, figsize=(9, 3.6), sharey=True)
    fields = (
        (axes[0], "risk_to_request_frames", "风险出现到请求 (帧)"),
        (axes[1], "request_to_decel_frames", "请求到实际减速 (帧)"),
    )
    for axis, field, title in fields:
        samples = []
        drawn = []
        for name in STRATEGIES:
            values = [row[field] for row in rows if row["strategy_name"] == name and row.get(field) is not None]
            if not values:
                continue
            samples.append(values)
            drawn.append(name)
        if samples:
            box = axis.boxplot(samples, tick_labels=[LABELS[name] for name in drawn], patch_artist=True)
            for patch, name in zip(box["boxes"], drawn):
                patch.set_facecolor(COLORS[name])
                patch.set_alpha(0.7)
        axis.set_title(title)
        axis.grid(True, axis="y", alpha=0.3)
    figure.tight_layout()
    return _save(figure, path)


def plot_outcome_grid(rows: list[dict], path: str | Path) -> Path:
    """每种策略一张格子。纵轴是场景的触发距离，颜色区分未碰撞和碰撞速度。"""
    _prepare()
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize
    from matplotlib.patches import Patch, Rectangle

    scenes = []
    for row in rows:
        key = (float(row["trigger_distance_m"]) if row.get("trigger_distance_m") is not None else 0.0, row["scene_id"])
        if key not in scenes:
            scenes.append(key)
    scenes.sort()
    repeats = sorted({row.get("repeat_index") for row in rows})
    speeds = [float(row["collision_relative_speed_mps"]) for row in rows if row["collided"] and row.get("collision_relative_speed_mps") is not None]
    limit = max(speeds) if speeds else 1.0
    norm = Normalize(vmin=0.0, vmax=limit)
    figure, axes = plt.subplots(1, len(STRATEGIES), figsize=(4.2 * len(STRATEGIES), max(2.8, 0.55 * len(scenes) + 1.4)), squeeze=False)
    lookup = {(row["scene_id"], row["strategy_name"], row.get("repeat_index")): row for row in rows}
    for axis, name in zip(axes[0], STRATEGIES):
        axis.set_xlim(-0.5, len(repeats) - 0.5)
        axis.set_ylim(-0.5, len(scenes) - 0.5)
        axis.set_xticks(range(len(repeats)), [f"重复 {item}" if item is not None else "单次" for item in repeats])
        axis.set_yticks(range(len(scenes)), [f"{scene}\n触发 {distance:.0f} m" for distance, scene in scenes])
        axis.set_title(LABELS[name])
        for row_index, (_distance, scene) in enumerate(scenes):
            for column, repeat in enumerate(repeats):
                item = lookup.get((scene, name, repeat))
                if item is None:
                    color = "#EEEEEE"
                    text = ""
                elif not item["collided"]:
                    color = "#43A047"
                    text = "未碰撞"
                else:
                    speed = item.get("collision_relative_speed_mps")
                    color = "#C62828" if speed is None else plt.cm.YlOrRd(norm(float(speed)))
                    text = "碰撞" if speed is None else f"{float(speed):.1f}"
                axis.add_patch(Rectangle((column - 0.5, row_index - 0.5), 1, 1, facecolor=color, edgecolor="white", linewidth=2))
                axis.text(column, row_index, text, ha="center", va="center", color="white", fontsize=8)
        axis.set_xlim(-0.5, len(repeats) - 0.5)
        axis.set_ylim(len(scenes) - 0.5, -0.5)
    figure.legend(
        handles=[Patch(facecolor="#43A047", label="未碰撞"), Patch(facecolor="#C62828", label="碰撞，颜色越深速度越高")],
        loc="lower center",
        ncol=2,
        frameon=False,
        bbox_to_anchor=(0.5, -0.02),
    )
    figure.tight_layout(rect=(0.0, 0.06, 1.0, 1.0))
    return _save(figure, path)


def _prepare() -> None:
    from viz.common import _prepare_matplotlib

    _prepare_matplotlib()
    import matplotlib

    matplotlib.use("Agg", force=True)


def _save(figure, path: str | Path) -> Path:
    import matplotlib.pyplot as plt

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(target, dpi=120, bbox_inches="tight")
    plt.close(figure)
    return target
