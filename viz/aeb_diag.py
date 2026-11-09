"""AEB 诊断图、三策略对比，以及从实际转移生成的状态图。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from control.delay import brake_onset
from scenarios.geometry import surface_gap
from viz.common import (
    AEB_STATE_COLORS,
    AEB_STATES,
    aeb_state_color,
    load_run,
    save_fig,
    strategy_color,
    _prepare_matplotlib,
)

_LABELS = {
    "no_intervention": "无干预",
    "fixed_ttc": "固定 TTC",
    "state_machine": "状态机",
}


def plot_aeb_diagnostic(run_dir: str | Path) -> Path:
    """单次运行：速度、TTC、状态色带，以及请求减速度和实测减速度。"""
    _prepare_matplotlib()
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    run = load_run(run_dir)
    frame_table = run.frames
    if frame_table.empty:
        raise ValueError(f"{run_dir} 没有帧，无法画诊断图")
    time_s = frame_table["sim_time_s"].to_numpy(dtype=float)
    speed = frame_table["ego_speed_mps"].to_numpy(dtype=float)
    ttc = frame_table["threat_ttc_s"].to_numpy(dtype=float)
    states = frame_table["aeb_state"].astype(str).tolist()
    requested = _requested(frame_table)
    measured = np.maximum(0.0, -frame_table["ego_a_long_mps2"].to_numpy(dtype=float))
    strategy_name = str(frame_table["strategy_name"].iloc[0])
    scene_id = str(run.manifest["scene_id"])
    delay = run.summary.get("brake_delay_frames")
    if delay is None:
        span = brake_onset(requested, measured)
        delay = None if span is None else span[1] - span[0]
    else:
        span = brake_onset(requested, measured)

    figure, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 8))
    speed_axis, ttc_axis, state_axis, decel_axis = axes
    color = strategy_color(strategy_name)
    speed_axis.plot(time_s, speed, color=color, linewidth=1.8)
    speed_axis.set_ylabel("速度 (m/s)")
    speed_axis.grid(True, alpha=0.3)
    ttc_axis.plot(time_s, ttc, color=color, linewidth=1.8)
    ttc_axis.set_ylabel("TTC (s)")
    ttc_axis.grid(True, alpha=0.3)
    widths = _widths(time_s)
    speed_axis.set_xlim(float(time_s[0]), float(time_s[-1] + widths[-1]))
    state_axis.bar(
        time_s,
        np.ones(len(time_s)),
        width=widths,
        color=[aeb_state_color(state) for state in states],
        align="edge",
        linewidth=0,
    )
    state_axis.set_ylabel("AEB")
    state_axis.set_ylim(0.0, 1.0)
    state_axis.set_yticks([])
    decel_axis.plot(time_s, requested, color="#C62828", linewidth=1.8, label="请求")
    decel_axis.plot(time_s, measured, color="#1565C0", linewidth=1.8, label="实测")
    decel_axis.set_ylabel("减速度 (m/s²)")
    decel_axis.set_xlabel("仿真时间 (s)")
    decel_axis.grid(True, alpha=0.3)
    decel_axis.legend(frameon=False, loc="upper right")
    if span is not None:
        decel_axis.axvline(time_s[span[0]], color="#C62828", linestyle="--", linewidth=1.0)
        decel_axis.axvline(time_s[span[1]], color="#1565C0", linestyle="--", linewidth=1.0)
    delay_text = "本段没有进入制动" if delay is None else f"请求到实测延迟 {int(delay)} 帧"
    decel_axis.text(0.01, 0.92, delay_text, transform=decel_axis.transAxes, fontsize=10)
    figure.legend(
        handles=[Patch(facecolor=AEB_STATE_COLORS[state], edgecolor="none", label=state) for state in AEB_STATES],
        loc="lower center",
        ncol=5,
        frameon=False,
        bbox_to_anchor=(0.5, 0.0),
    )
    figure.suptitle(f"{scene_id} / {_LABELS.get(strategy_name, strategy_name)}")
    figure.tight_layout(rect=(0.0, 0.05, 1.0, 0.96))
    path = save_fig(figure, run_dir, "aeb_diagnostic")
    plt.close(figure)
    return path


def write_state_diagram(run_dir: str | Path) -> tuple[Path, Path]:
    """按本段实际转移写 Mermaid，并画一张只含经过状态的图。"""
    _prepare_matplotlib()
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyBboxPatch

    root = Path(run_dir)
    events = _read_events(root / "events.jsonl")
    edges = []
    visited = ["IDLE"]
    for event in events:
        if event.get("event_type") != "aeb_state":
            continue
        detail = event.get("detail") or {}
        previous = str(detail.get("from", ""))
        current = str(detail.get("to", ""))
        reason = str(detail.get("reason", ""))
        edge = (previous, current, reason)
        if edge not in edges:
            edges.append(edge)
        if current not in visited:
            visited.append(current)
    lines = ["stateDiagram-v2"]
    if edges:
        lines.extend(f"    {previous} --> {current}: {reason}" for previous, current, reason in edges)
    else:
        lines.append("    IDLE")
    mermaid = root / "aeb_states.mmd"
    mermaid.write_text("\n".join(lines) + "\n", encoding="utf-8")

    figure, axis = plt.subplots(figsize=(max(6.0, 1.6 * len(visited)), 3.2), dpi=120)
    axis.set_xlim(-0.2, len(visited) - 0.2)
    axis.set_ylim(-0.8, 1.6)
    axis.axis("off")
    positions = {}
    for index, state in enumerate(visited):
        positions[state] = index
        color = aeb_state_color(state)
        patch = FancyBboxPatch(
            (index - 0.38, 0.55),
            0.76,
            0.5,
            boxstyle="round,pad=0.04,rounding_size=0.08",
            facecolor=color,
            edgecolor=color,
        )
        axis.add_patch(patch)
        axis.text(index, 0.8, state, ha="center", va="center", color="white", fontsize=10)
    for previous, current, reason in edges:
        if previous not in positions or current not in positions:
            continue
        start = positions[previous]
        end = positions[current]
        axis.annotate(
            "",
            xy=(end, 0.42),
            xytext=(start, 0.42),
            arrowprops={"arrowstyle": "->", "color": "#424242", "lw": 1.0},
        )
        axis.text((start + end) * 0.5, 0.12, reason, ha="center", va="bottom", fontsize=8, color="#424242")
    scene_id = _scene_id(root)
    axis.set_title(f"{scene_id} 实际经过的 AEB 状态")
    figure.tight_layout()
    png = save_fig(figure, root, "aeb_states")
    plt.close(figure)
    return mermaid, png


def plot_strategy_compare(run_dirs: list[str | Path], output: str | Path) -> Path:
    """同一场景的多条策略画在一张图上。颜色按策略名固定。"""
    _prepare_matplotlib()
    import matplotlib.pyplot as plt

    if not run_dirs:
        raise ValueError("对比至少要有一次运行")
    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    speed_axis, gap_axis = axes
    scene_id = ""
    for run_dir in run_dirs:
        run = load_run(run_dir)
        scene_id = str(run.manifest["scene_id"])
        frame_table = run.frames
        time_s = frame_table["sim_time_s"].to_numpy(dtype=float)
        speed = frame_table["ego_speed_mps"].to_numpy(dtype=float)
        name = str(frame_table["strategy_name"].iloc[0])
        color = strategy_color(name)
        label = _LABELS.get(name, name)
        speed_axis.plot(time_s, speed, color=color, linewidth=1.8, label=label)
        gaps = _surface_gaps(run)
        gap_axis.plot(time_s, gaps, color=color, linewidth=1.8, label=label)
        onset = _brake_onset_index(frame_table)
        if onset is not None:
            speed_axis.scatter([time_s[onset]], [speed[onset]], color=color, marker="v", s=36, zorder=4)
            gap_axis.scatter([time_s[onset]], [gaps[onset]], color=color, marker="v", s=36, zorder=4)
    speed_axis.set_ylabel("速度 (m/s)")
    speed_axis.grid(True, alpha=0.3)
    speed_axis.legend(frameon=False, loc="best")
    gap_axis.set_ylabel("表面间距 (m)")
    gap_axis.set_xlabel("仿真时间 (s)")
    gap_axis.grid(True, alpha=0.3)
    figure.suptitle(f"{scene_id} 策略对比")
    figure.tight_layout()
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(figure)
    return path


def _requested(frame_table) -> np.ndarray:
    column = "control_request_target_decel_mps2"
    if column not in frame_table.columns:
        return np.zeros(len(frame_table), dtype=float)
    values = frame_table[column].to_numpy(dtype=float)
    return np.nan_to_num(values, nan=0.0)


def _widths(time_s: np.ndarray) -> np.ndarray:
    if len(time_s) == 1:
        return np.array([0.05])
    steps = np.diff(time_s)
    return np.append(steps, steps[-1])


def _brake_onset_index(frame_table) -> int | None:
    requested = _requested(frame_table)
    states = frame_table["aeb_state"].astype(str).tolist()
    for index, (value, state) in enumerate(zip(requested, states)):
        if value > 0.1 or state in {"PARTIAL", "FULL"}:
            return index
    return None


def _surface_gaps(run) -> np.ndarray:
    scenario = run.manifest.get("scenario") or {}
    ego_box = scenario.get("ego") or {}
    target_box = scenario.get("target") or {}
    targets = {}
    if not run.targets.empty:
        for row in run.targets.to_dict("records"):
            targets.setdefault(int(row["frame"]), []).append(row)
    gaps = []
    for row in run.frames.to_dict("records"):
        group = targets.get(int(row["frame"]), [])
        if not group:
            gaps.append(float("nan"))
            continue
        best = None
        for item in group:
            gap = surface_gap(
                float(row["ego_x_m"]),
                float(row["ego_y_m"]),
                float(row["ego_yaw_rad"]),
                float(ego_box["length_m"]),
                float(ego_box["width_m"]),
                float(item["track_x_m"]),
                float(item["track_y_m"]),
                float(item["track_yaw_rad"]),
                float(target_box["length_m"]),
                float(target_box["width_m"]),
            )
            best = gap if best is None else min(best, gap)
        gaps.append(float(best))
    return np.asarray(gaps, dtype=float)


def _read_events(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def _scene_id(root: Path) -> str:
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    return str(manifest.get("scene_id", root.name))
