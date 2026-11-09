"""威胁时间线和某一帧的俯视快照。只读运行目录。"""

from __future__ import annotations

import math

import numpy as np
import pandas as pd

from scenarios.geometry import box_corners, forward_xy, right_xy
from threat.assessor import REASON_TEXT
from viz.common import load_run, save_fig, strategy_color, _prepare_matplotlib


def plot_threat_timeline(run_dir) -> "object":
    """TTC、预测最小间距、制动可避免性。风险无效的时段铺灰并写原因。"""
    _prepare_matplotlib()
    import matplotlib.pyplot as plt

    run = load_run(run_dir)
    frame_table = run.frames
    if frame_table.empty:
        raise ValueError(f"{run_dir} 没有帧，无法画威胁时间线")
    time_s = frame_table["sim_time_s"].to_numpy(dtype=float)
    ttc = _numeric(frame_table, "threat_ttc_s")
    gap = _numeric(frame_table, "threat_pred_min_distance_m")
    avoid = _avoid_flag(frame_table)
    reasons = _reasons(frame_table)
    strategy_name = str(frame_table["strategy_name"].iloc[0])
    scene_id = str(run.manifest["scene_id"])
    color = strategy_color(strategy_name)

    figure, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
    ttc_axis, gap_axis, avoid_axis = axes
    ttc_axis.plot(time_s, ttc, color=color, linewidth=1.8)
    ttc_axis.set_ylabel("TTC (s)")
    gap_axis.plot(time_s, gap, color=color, linewidth=1.8)
    gap_axis.set_ylabel("预测最小间距 (m)")
    avoid_axis.step(time_s, avoid, where="post", color=color, linewidth=1.8)
    avoid_axis.set_ylabel("制动可避免")
    avoid_axis.set_ylim(-0.1, 1.1)
    avoid_axis.set_yticks([0, 1])
    avoid_axis.set_yticklabels(["否", "是"])
    avoid_axis.set_xlabel("仿真时间 (s)")
    spans = _invalid_spans(time_s, reasons)
    for axis in axes:
        axis.grid(True, alpha=0.3)
        for start, end, _reason in spans:
            axis.axvspan(start, end, color="#BDBDBD", alpha=0.35, linewidth=0)
    _annotate_reasons(ttc_axis, spans, time_s)
    figure.suptitle(f"{scene_id} / {strategy_name} 威胁")
    figure.tight_layout()
    path = save_fig(figure, run_dir, "threat_timeline")
    plt.close(figure)
    return path


def plot_threat_snapshot(run_dir, frame: int | None = None):
    """选定一帧的俯视图：包围盒、匀速预测轨迹、冲突点。主威胁用红色。"""
    _prepare_matplotlib()
    import matplotlib.pyplot as plt

    run = load_run(run_dir)
    frame_table = run.frames
    if frame_table.empty:
        raise ValueError(f"{run_dir} 没有帧，无法画威胁快照")
    chosen = _choose_frame(frame_table, frame)
    row = frame_table.loc[frame_table["frame"] == chosen].iloc[0]
    ego_length, ego_width = _ego_size(run.manifest)
    figure, axis = plt.subplots(figsize=(7.2, 7.2))
    _draw_box(
        axis,
        float(row["ego_x_m"]),
        float(row["ego_y_m"]),
        float(row["ego_yaw_rad"]),
        ego_length,
        ego_width,
        "#1565C0",
        "自车",
    )
    _draw_velocity(
        axis,
        float(row["ego_x_m"]),
        float(row["ego_y_m"]),
        float(row["ego_yaw_rad"]),
        float(row["ego_v_long_mps"]),
        float(row["ego_v_lat_mps"]),
        "#1565C0",
    )
    threat_id = row.get("threat_threat_id")
    if isinstance(threat_id, float) and math.isnan(threat_id):
        threat_id = None
    targets = _targets_at(run.targets, chosen)
    for target in targets:
        primary = threat_id is not None and str(target["track_id"]) == str(threat_id)
        color = "#E53935" if primary else "#90A4AE"
        _draw_box(
            axis,
            target["x_m"],
            target["y_m"],
            target["yaw_rad"],
            target["length_m"],
            target["width_m"],
            color,
            "主威胁" if primary else str(target["track_id"]),
        )
        _draw_velocity(
            axis,
            target["x_m"],
            target["y_m"],
            target["yaw_rad"],
            target["v_long_mps"],
            target["v_lat_mps"],
            color,
        )
        _draw_prediction(axis, target, color)
    conflict_x = row.get("threat_conflict_x_m")
    conflict_y = row.get("threat_conflict_y_m")
    if _finite(conflict_x) and _finite(conflict_y):
        axis.scatter([float(conflict_x)], [float(conflict_y)], marker="X", s=80, color="#212121", zorder=5, label="冲突点")
    ttc = row.get("threat_ttc_s")
    ttc_text = "无" if not _finite(ttc) else f"{float(ttc):.2f} s"
    axis.relim()
    axis.autoscale_view()
    axis.margins(0.15)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.3)
    axis.set_xlabel("世界 X (m)")
    axis.set_ylabel("世界 Y (m)")
    axis.set_title(f"帧 {chosen}  TTC {ttc_text}")
    axis.legend(loc="best", frameon=False)
    figure.tight_layout()
    path = save_fig(figure, run_dir, f"threat_snapshot_{chosen}")
    plt.close(figure)
    return path


def _numeric(frame_table: pd.DataFrame, column: str) -> np.ndarray:
    if column not in frame_table.columns:
        return np.full(len(frame_table), np.nan)
    return pd.to_numeric(frame_table[column], errors="coerce").to_numpy(dtype=float)


def _avoid_flag(frame_table: pd.DataFrame) -> np.ndarray:
    if "threat_brake_avoidable" not in frame_table.columns:
        return np.full(len(frame_table), np.nan)
    values = []
    for item in frame_table["threat_brake_avoidable"].tolist():
        if item is True or item == 1:
            values.append(1.0)
        elif item is False or item == 0:
            values.append(0.0)
        else:
            values.append(np.nan)
    return np.asarray(values, dtype=float)


def _reasons(frame_table: pd.DataFrame) -> list[str | None]:
    if "threat_invalid_reason" not in frame_table.columns:
        return [None] * len(frame_table)
    reasons = []
    for item in frame_table["threat_invalid_reason"].tolist():
        if item is None or (isinstance(item, float) and math.isnan(item)) or pd.isna(item):
            reasons.append(None)
        else:
            text = str(item).strip()
            reasons.append(text or None)
    return reasons


def _invalid_spans(time_s: np.ndarray, reasons: list[str | None]) -> list[tuple[float, float, str]]:
    if len(time_s) == 0:
        return []
    step = float(time_s[1] - time_s[0]) if len(time_s) > 1 else 0.05
    spans = []
    start = None
    label = None
    for index, (moment, reason) in enumerate(zip(time_s, reasons)):
        if reason == label and start is not None:
            continue
        if start is not None and label is not None:
            spans.append((start, float(time_s[index]), label))
        start = float(moment) if reason else None
        label = reason
    if start is not None and label is not None:
        spans.append((start, float(time_s[-1]) + step, label))
    return spans


def _annotate_reasons(axis, spans, time_s: np.ndarray) -> None:
    if len(time_s) == 0 or not spans:
        return
    _bottom, top = axis.get_ylim()
    for start, end, reason in spans:
        if end - start < (time_s[-1] - time_s[0]) * 0.02 and len(spans) > 4:
            continue
        axis.text(
            (start + end) * 0.5,
            top,
            REASON_TEXT.get(reason, reason),
            ha="center",
            va="bottom",
            fontsize=8,
            color="#424242",
        )


def _choose_frame(frame_table: pd.DataFrame, frame: int | None) -> int:
    frames = frame_table["frame"].astype(int)
    if frame is not None:
        if int(frame) not in set(frames.tolist()):
            raise ValueError(f"没有帧 {frame}")
        return int(frame)
    valid = frame_table[pd.to_numeric(frame_table["threat_ttc_s"], errors="coerce").notna()]
    if valid.empty:
        return int(frames.iloc[len(frames) // 2])
    ttc = pd.to_numeric(valid["threat_ttc_s"], errors="coerce")
    readable = valid.loc[ttc >= 0.2]
    pool = readable if not readable.empty else valid
    distance = (pd.to_numeric(pool["threat_ttc_s"], errors="coerce") - 1.0).abs()
    return int(pool.loc[distance.idxmin(), "frame"])


def _ego_size(manifest: dict) -> tuple[float, float]:
    ego = ((manifest.get("scenario") or {}).get("ego") or {})
    length = float(ego.get("length_m", 4.9))
    width = float(ego.get("width_m", 2.12))
    return length, width


def _targets_at(targets: pd.DataFrame, frame: int) -> list[dict]:
    if targets.empty or "frame" not in targets.columns:
        return []
    rows = targets.loc[targets["frame"] == frame]
    found = []
    for _, row in rows.iterrows():
        found.append(
            {
                "track_id": row.get("track_track_id", ""),
                "x_m": float(row["track_x_m"]),
                "y_m": float(row["track_y_m"]),
                "yaw_rad": float(row["track_yaw_rad"]),
                "length_m": float(row["track_length_m"]),
                "width_m": float(row["track_width_m"]),
                "v_long_mps": float(row["track_v_long_mps"]),
                "v_lat_mps": float(row["track_v_lat_mps"]),
            }
        )
    return found


def _draw_box(axis, x_m, y_m, yaw_rad, length_m, width_m, color, label) -> None:
    from matplotlib.patches import Polygon

    corners = box_corners(x_m, y_m, yaw_rad, length_m, width_m)
    axis.add_patch(Polygon(corners, closed=True, facecolor=color, edgecolor=color, alpha=0.35, linewidth=1.5, label=label))


def _draw_velocity(axis, x_m, y_m, yaw_rad, v_long_mps, v_lat_mps, color) -> None:
    vx, vy = _world_velocity(yaw_rad, v_long_mps, v_lat_mps)
    scale = 0.4
    if math.hypot(vx, vy) < 1e-6:
        return
    axis.annotate(
        "",
        xy=(x_m + vx * scale, y_m + vy * scale),
        xytext=(x_m, y_m),
        arrowprops={"arrowstyle": "->", "color": color, "lw": 1.4},
    )


def _draw_prediction(axis, target: dict, color: str) -> None:
    vx, vy = _world_velocity(target["yaw_rad"], target["v_long_mps"], target["v_lat_mps"])
    times = np.linspace(0.0, 3.0, 16)
    axis.plot(
        target["x_m"] + vx * times,
        target["y_m"] + vy * times,
        linestyle="--",
        color=color,
        linewidth=1.2,
    )


def _world_velocity(yaw_rad: float, v_long_mps: float, v_lat_mps: float) -> tuple[float, float]:
    fx, fy = forward_xy(yaw_rad)
    rx, ry = right_xy(yaw_rad)
    return fx * v_long_mps + rx * v_lat_mps, fy * v_long_mps + ry * v_lat_mps


def _finite(value) -> bool:
    if value is None or isinstance(value, bool):
        return False
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(number)
