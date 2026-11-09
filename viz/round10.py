"""第 10 轮的难例图、邻域图、回归色表和回放关键帧。"""

from __future__ import annotations

import base64
from pathlib import Path

from round10.mine import CAUSE_TEXT
from scenarios.geometry import box_corners
from viz.common import _prepare_matplotlib


CAUSE_COLOR = {
    "late_brake": "#C62828",
    "late_discovery": "#EF6C00",
    "slow_response": "#6D4C41",
    "false_trigger": "#8E24AA",
    "aes_infeasible": "#1565C0",
    "tracking_error": "#00897B",
    "data_gap": "#546E7A",
    "unspecified": "#B0BEC5",
}
STATUS_COLOR = {"new_failure": "#C62828", "fixed": "#2E7D32", "unchanged": "#B0BEC5"}
STATUS_TEXT = {"new_failure": "新增失败", "fixed": "已修复", "unchanged": "未变"}


def plot_causes(cases: list[dict], path: Path) -> None:
    _prepare_matplotlib()
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    counts: dict[str, int] = {}
    for case in cases:
        counts[case["suspected_cause"]] = counts.get(case["suspected_cause"], 0) + 1
    labels = list(counts)
    figure, axes = plt.subplots(1, 2, figsize=(11.2, 4.6), constrained_layout=True)
    axes[0].bar(range(len(labels)), [counts[item] for item in labels], color=[CAUSE_COLOR.get(item, "#546E7A") for item in labels])
    axes[0].set_xticks(range(len(labels)), [CAUSE_TEXT.get(item, item) for item in labels], rotation=20, ha="right")
    axes[0].set_ylabel("条数")
    axes[0].set_title("疑似原因线索")
    strategies = []
    for case in cases:
        if case["strategy_name"] not in strategies:
            strategies.append(case["strategy_name"])
    for case in cases:
        shift = (strategies.index(case["strategy_name"]) - (len(strategies) - 1) / 2) * 0.22
        axes[1].scatter(
            case["ego_speed_mps"] + shift,
            case["distance_m"],
            s=36,
            color=CAUSE_COLOR.get(case["suspected_cause"], "#546E7A"),
            alpha=0.8,
        )
    axes[1].set_xlabel("自车速度 (m/s)")
    axes[1].set_ylabel("初始距离 (m)")
    axes[1].set_title("难例在速度-距离上的位置")
    from matplotlib.lines import Line2D

    axes[1].legend(
        handles=[
            Line2D([0], [0], marker="o", color="none", markerfacecolor=CAUSE_COLOR.get(item, "#546E7A"), label=CAUSE_TEXT.get(item, item), markersize=7)
            for item in labels
        ],
        fontsize=8,
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=140)
    plt.close(figure)


def plot_neighborhood(cases: list[dict], generated: list[dict], path: Path) -> None:
    _prepare_matplotlib()
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(7.2, 5.2), constrained_layout=True)
    if generated:
        axis.scatter(
            [item["ego_speed_mps"] for item in generated],
            [item["distance_m"] for item in generated],
            s=28,
            marker="s",
            color="#1565C0",
            label="新生成",
        )
    axis.scatter(
        [item["ego_speed_mps"] for item in cases],
        [item["distance_m"] for item in cases],
        s=64,
        marker="o",
        color="#C62828",
        label="原难例",
        zorder=3,
    )
    axis.set_xlabel("自车速度 (m/s)")
    axis.set_ylabel("初始距离 (m)")
    axis.set_title("邻域加密前后")
    axis.legend()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=140)
    plt.close(figure)


def plot_regression(rows: list[dict], path: Path) -> None:
    _prepare_matplotlib()
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 2, figsize=(11.2, 4.4), constrained_layout=True)
    table_axis = axes[0]
    table_axis.set_axis_off()
    if rows:
        cell = [[STATUS_TEXT[row["status"]]] for row in rows]
        colours = [[STATUS_COLOR[row["status"]]] for row in rows]
        table = table_axis.table(cellText=cell, rowLabels=[row["case_id"] for row in rows], colLabels=["差异"], cellColours=colours, loc="center")
        table.auto_set_font_size(False)
        table.set_fontsize(8)
        table.scale(1.0, 1.3)
    table_axis.set_title("回归差异")
    counts = {key: 0 for key in STATUS_TEXT}
    for row in rows:
        counts[row["status"]] += 1
    axes[1].bar(range(3), [counts[key] for key in STATUS_TEXT], color=[STATUS_COLOR[key] for key in STATUS_TEXT])
    axes[1].set_xticks(range(3), [STATUS_TEXT[key] for key in STATUS_TEXT])
    axes[1].set_ylabel("条数")
    axes[1].set_title("差异汇总")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=140)
    plt.close(figure)


def export_replay(result, case_id: str, directory: Path) -> dict:
    """俯视动画，外加风险出现、制动开始、最近距离三张静帧。"""
    _prepare_matplotlib()
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    directory.mkdir(parents=True, exist_ok=True)
    samples = result.samples or []
    keys = {
        "risk": _frame_of(result, "risk"),
        "brake": _frame_of(result, "brake"),
        "closest": _frame_of(result, "closest"),
    }
    titles = {"risk": "风险出现", "brake": "制动开始", "closest": "最近点"}
    written = {}
    for name, frame in keys.items():
        path = directory / f"{case_id}_{name}.png"
        _draw_frame(samples, frame, titles[name], path)
        written[name] = path
    gif_path = directory / f"{case_id}_bev.gif"
    figure, axis = plt.subplots(figsize=(6.2, 4.4))
    step = 2 if len(samples) > 40 else 1
    shown = samples[::step] or samples

    def update(index):
        axis.clear()
        _paint(axis, shown[index], f"{case_id}  {shown[index]['sim_time_s']:.2f} s")
        return []

    animation = FuncAnimation(figure, update, frames=len(shown), blit=False)
    animation.save(gif_path, writer=PillowWriter(fps=8))
    plt.close(figure)
    written["bev"] = gif_path
    return written


def _frame_of(result, kind: str) -> int | None:
    if kind == "risk":
        return result.first_ttc_frame
    if kind == "brake":
        for record in result.records:
            decel = record.control_request.target_decel_mps2
            if decel is not None and decel >= 0.5:
                return record.frame
        return None
    if result.collision_frame is not None:
        return result.collision_frame
    from round9.simulate import _gap

    best_frame = None
    best_gap = None
    for sample in result.samples or []:
        ego = sample["ego"]
        if not sample["bodies"]:
            continue
        gap = min(_gap(ego["x_m"], ego["y_m"], ego["yaw_rad"], _body(body)) for body in sample["bodies"])
        if best_gap is None or gap < best_gap:
            best_gap = gap
            best_frame = sample["frame"]
    return best_frame


def _body(raw):
    from round9.simulate import Body

    return Body(raw["track_id"], "vehicle", raw["x_m"], raw["y_m"], raw["yaw_rad"], 0.0, 0.0, raw["length_m"], raw["width_m"], 1.0)


def _draw_frame(samples, frame, title: str, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    sample = next((item for item in samples if item["frame"] == frame), None)
    figure, axis = plt.subplots(figsize=(6.2, 4.4), constrained_layout=True)
    if sample is None:
        axis.set_title(f"{title}：这一帧没有样本")
    else:
        _paint(axis, sample, title)
    figure.savefig(path, dpi=140)
    plt.close(figure)


def _paint(axis, sample, title: str) -> None:
    _box(axis, sample["ego"], "#1565C0")
    for body in sample["bodies"]:
        _box(axis, body, "#C62828")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("X (m)")
    axis.set_ylabel("Y (m)")
    axis.set_title(title)
    axis.grid(True, linewidth=0.3)


def _box(axis, body, color: str) -> None:
    corners = box_corners(body["x_m"], body["y_m"], body["yaw_rad"], body["length_m"], body["width_m"])
    closed = list(corners) + [corners[0]]
    axis.plot([item[0] for item in closed], [item[1] for item in closed], color=color, linewidth=1.4)


def write_report(path: Path, chapters: list[tuple[str, str, list[Path]]]) -> None:
    parts = [
        "<!DOCTYPE html><html lang='zh-CN'><head><meta charset='utf-8'><title>第 10 轮</title>",
        "<style>body{font-family:'Microsoft YaHei',sans-serif;margin:32px;color:#212121;}",
        "table{border-collapse:collapse;margin:12px 0 24px;}td,th{border:1px solid #B0BEC5;padding:6px 10px;}",
        "img{max-width:100%;margin:8px 0 20px;}h2{margin-top:28px;}</style></head><body>",
        "<h1>第 10 轮 难例挖掘和回归</h1>",
    ]
    for title, html, images in chapters:
        parts.append(f"<h2>{title}</h2>")
        parts.append(html)
        for image in images:
            if image.suffix.lower() == ".gif":
                continue
            encoded = base64.b64encode(image.read_bytes()).decode("ascii")
            parts.append(f"<img alt='{image.name}' src='data:image/png;base64,{encoded}'>")
    parts.append("</body></html>")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(parts), encoding="utf-8")
