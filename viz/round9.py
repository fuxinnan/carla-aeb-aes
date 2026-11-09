"""第 9 轮的图和报告。调参集和测试集分开放，不混在一张图里。"""

from __future__ import annotations

import base64
import math
from pathlib import Path

from viz.common import _prepare_matplotlib


TEMPLATE_TEXT = {
    "vehicle_cut_in": "车辆切入",
    "occlusion_reveal": "遮挡后出现",
    "left_avoidable": "左侧可避让",
    "right_avoidable": "右侧可避让",
    "both_unavoidable": "两侧不可避让",
    "parallel_vehicle": "旁车并行",
}
STRATEGY_TEXT = {
    "no_intervention": "无干预",
    "fixed_ttc": "固定 TTC",
    "state_machine": "状态机 AEB",
    "aeb_aes": "状态机 AEB+AES",
}
STRATEGY_COLOR = {
    "no_intervention": "#546E7A",
    "fixed_ttc": "#C62828",
    "state_machine": "#1565C0",
    "aeb_aes": "#8E24AA",
}


def plot_heatmaps(rows: list[dict], directory: Path, split: str, strategies: tuple[str, ...], templates: tuple[str, ...]) -> list[Path]:
    """每个策略一张图，每格是一个模板。颜色是碰撞率。"""
    _prepare_matplotlib()
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    directory.mkdir(parents=True, exist_ok=True)
    written = []
    speeds = sorted({row["ego_speed_mps"] for row in rows})
    distances = sorted({row["distance_m"] for row in rows})
    for strategy in strategies:
        figure, axes = plt.subplots(2, 3, figsize=(11.2, 6.4), constrained_layout=True)
        for axis, template in zip(axes.ravel(), templates):
            grid = [[math.nan for _ in speeds] for _ in distances]
            for row in rows:
                if row["strategy_name"] != strategy or row["template"] != template or row["split"] != split:
                    continue
                grid[distances.index(row["distance_m"])][speeds.index(row["ego_speed_mps"])] = 1.0 if row["collided"] else 0.0
            cmap = plt.get_cmap("Reds").copy()
            cmap.set_bad("#B0BEC5")
            image = axis.imshow(grid, vmin=0.0, vmax=1.0, cmap=cmap, origin="lower", aspect="auto")
            axis.set_xticks(range(len(speeds)), [f"{speed:.0f}" for speed in speeds])
            axis.set_yticks(range(len(distances)), [f"{distance:.0f}" for distance in distances])
            axis.set_xlabel("自车速度 (m/s)")
            axis.set_ylabel("初始距离 (m)")
            axis.set_title(TEMPLATE_TEXT.get(template, template))
            for row_index, distance in enumerate(distances):
                for col_index, speed in enumerate(speeds):
                    value = grid[row_index][col_index]
                    if math.isnan(value):
                        continue
                    axis.text(col_index, row_index, f"{value:.0f}", ha="center", va="center", color="#212121", fontsize=9)
        figure.colorbar(image, ax=axes.ravel().tolist(), shrink=0.8, label="碰撞率")
        figure.suptitle(f"{'调参集' if split == 'tuning' else '测试集'} · {STRATEGY_TEXT.get(strategy, strategy)}")
        path = directory / f"grid_{split}_{strategy}.png"
        figure.savefig(path, dpi=140)
        plt.close(figure)
        written.append(path)
    return written


def plot_ablation(rows: list[dict], labels: list[tuple[str, str]], path: Path) -> None:
    """完整状态机和三次去掉部件。指标是碰撞率、误制动率、状态切换次数。"""
    _prepare_matplotlib()
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    keys = ("collision_rate", "false_brake_rate", "state_switch_count")
    names = ("碰撞率", "硬负例误制动率", "平均状态切换次数")
    figure, axes = plt.subplots(1, 3, figsize=(11.2, 4.2), constrained_layout=True)
    colors = ["#1565C0", "#EF6C00", "#6D4C41", "#00897B"]
    for axis, key, name in zip(axes, keys, names):
        values = []
        tick = []
        for token, label in labels:
            group = [row for row in rows if row.get("ablation") == token]
            tick.append(label)
            if key == "collision_rate":
                values.append(sum(1.0 for row in group if row["collided"]) / len(group))
            elif key == "false_brake_rate":
                hard = [row for row in group if row["hard_negative"]]
                values.append(0.0 if not hard else sum(1.0 for row in hard if row["false_brake"]) / len(hard))
            else:
                values.append(sum(row["state_switch_count"] for row in group) / len(group))
        axis.bar(range(len(values)), values, color=colors[: len(values)])
        axis.set_xticks(range(len(tick)), tick, rotation=20, ha="right")
        axis.set_ylabel(name)
        axis.set_title(name)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=140)
    plt.close(figure)


def plot_stability(rows: list[dict], near: set[tuple[str, str]], path: Path) -> None:
    """每次重复的碰撞结果。靠近碰撞边界的点用橙色。"""
    _prepare_matplotlib()
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    strategies = []
    for row in rows:
        if row["strategy_name"] not in strategies:
            strategies.append(row["strategy_name"])
    figure, axes = plt.subplots(len(strategies), 1, figsize=(11.2, 2.2 * len(strategies)), constrained_layout=True, sharex=True)
    if len(strategies) == 1:
        axes = [axes]
    scenes = []
    for row in rows:
        if row["scene_id"] not in scenes:
            scenes.append(row["scene_id"])
    for axis, strategy in zip(axes, strategies):
        for row in rows:
            if row["strategy_name"] != strategy:
                continue
            index = scenes.index(row["scene_id"])
            repeat = 0 if row["repeat_index"] is None else int(row["repeat_index"])
            marked = (row["scene_id"], strategy) in near
            axis.scatter(
                index + (repeat - 1) * 0.12,
                1.0 if row["collided"] else 0.0,
                s=28,
                color="#EF6C00" if marked else STRATEGY_COLOR.get(strategy, "#1565C0"),
                zorder=3,
            )
        axis.set_yticks([0, 1], ["未碰撞", "碰撞"])
        axis.set_ylabel(STRATEGY_TEXT.get(strategy, strategy))
        axis.set_ylim(-0.3, 1.3)
    axes[-1].set_xticks(range(len(scenes)), scenes, rotation=30, ha="right", fontsize=7)
    axes[0].set_title("重复批次的结果。橙色是靠近碰撞边界的场景")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=140)
    plt.close(figure)


def _or_zero(value) -> float:
    return 0.0 if value is None else float(value)


def plot_occlusion(pairs: list[dict], path: Path) -> None:
    """同一条遮挡场景，全可见和遮挡可见的风险出现帧、碰撞率并排。"""
    _prepare_matplotlib()
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    labels = [item["label"] for item in pairs]
    figure, axes = plt.subplots(1, 2, figsize=(10.4, 4.2), constrained_layout=True)
    x = list(range(len(pairs)))
    width = 0.36
    axes[0].bar([item - width / 2 for item in x], [_or_zero(item["full_frame"]) for item in pairs], width=width, color="#546E7A", label="真值全可见")
    axes[0].bar([item + width / 2 for item in x], [_or_zero(item["occluded_frame"]) for item in pairs], width=width, color="#EF6C00", label="遮挡可见")
    axes[0].set_ylabel("行人第一次出现有效 TTC 的帧")
    axes[0].set_title("风险出现帧")
    axes[0].legend()
    axes[1].bar([item - width / 2 for item in x], [item["full_collision"] for item in pairs], width=width, color="#546E7A", label="真值全可见")
    axes[1].bar([item + width / 2 for item in x], [item["occluded_collision"] for item in pairs], width=width, color="#EF6C00", label="遮挡可见")
    axes[1].set_ylabel("碰撞（1 为碰撞）")
    axes[1].set_ylim(0, 1.2)
    axes[1].set_title("碰撞")
    axes[1].legend()
    for axis in axes:
        axis.set_xticks(x, labels, rotation=15, ha="right")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=140)
    plt.close(figure)


def write_report(path: Path, chapters: list[tuple[str, str, list[Path]]]) -> None:
    """章节标题、说明和嵌进去的图。"""
    parts = [
        "<!DOCTYPE html>",
        "<html lang='zh-CN'><head><meta charset='utf-8'><title>第 9 轮</title>",
        "<style>body{font-family:'Microsoft YaHei',sans-serif;margin:32px;color:#212121;}",
        "table{border-collapse:collapse;margin:12px 0 24px;}td,th{border:1px solid #B0BEC5;padding:6px 10px;text-align:left;}",
        "img{max-width:100%;margin:8px 0 20px;}h2{margin-top:28px;}</style></head><body>",
        "<h1>第 9 轮 场景扩充、调参、消融和稳定性</h1>",
    ]
    for title, html, images in chapters:
        parts.append(f"<h2>{title}</h2>")
        parts.append(html)
        for image in images:
            encoded = base64.b64encode(image.read_bytes()).decode("ascii")
            parts.append(f"<img alt='{image.name}' src='data:image/png;base64,{encoded}'>")
    parts.append("</body></html>")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(parts), encoding="utf-8")
