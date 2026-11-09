"""自车时间曲线和世界系俯视轨迹。只读已经写好的运行目录。"""

from __future__ import annotations

import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if not any(Path(entry).resolve() == _PROJECT_ROOT for entry in sys.path if entry):
    sys.path.insert(0, str(_PROJECT_ROOT))

from viz.common import _prepare_matplotlib, load_run, save_fig, strategy_color


def plot_ego_run(run_dir: str | Path) -> Path:
    """速度、加速度、位置随仿真时间的曲线，以及世界坐标俯视图。"""
    _prepare_matplotlib()
    import matplotlib.pyplot as plt

    run = load_run(run_dir)
    frames = run.frames
    if frames.empty:
        raise ValueError(f"{run_dir} 没有帧，无法画出自车轨迹")
    time_s = frames["sim_time_s"].to_numpy(dtype=float)
    speed = frames["ego_speed_mps"].to_numpy(dtype=float)
    accel = frames["ego_a_long_mps2"].to_numpy(dtype=float)
    x_m = frames["ego_x_m"].to_numpy(dtype=float)
    y_m = frames["ego_y_m"].to_numpy(dtype=float)
    z_m = frames["ego_z_m"].to_numpy(dtype=float)
    color = strategy_color(str(frames["strategy_name"].iloc[0]))
    scene_id = str(run.manifest["scene_id"])
    collision_xy = _collision_points(run.events, frames)

    figure, axes = plt.subplots(2, 2, figsize=(10, 8))
    speed_axis, accel_axis, position_axis, bev_axis = axes.ravel()

    speed_axis.plot(time_s, speed, color=color, linewidth=1.6)
    speed_axis.set_xlabel("仿真时间 (s)")
    speed_axis.set_ylabel("速度 (m/s)")
    speed_axis.grid(True, alpha=0.3)

    accel_axis.plot(time_s, accel, color=color, linewidth=1.6)
    accel_axis.set_xlabel("仿真时间 (s)")
    accel_axis.set_ylabel("纵向加速度 (m/s²)")
    accel_axis.grid(True, alpha=0.3)

    position_axis.plot(time_s, x_m, label="X", linewidth=1.4)
    position_axis.plot(time_s, y_m, label="Y", linewidth=1.4)
    position_axis.plot(time_s, z_m, label="Z", linewidth=1.4)
    position_axis.set_xlabel("仿真时间 (s)")
    position_axis.set_ylabel("位置 (m)")
    position_axis.legend(frameon=False)
    position_axis.grid(True, alpha=0.3)

    bev_axis.plot(x_m, y_m, color=color, linewidth=1.6)
    bev_axis.scatter([x_m[0]], [y_m[0]], color="#43A047", s=36, label="起点", zorder=3)
    if collision_xy:
        bev_axis.scatter(
            [point[0] for point in collision_xy],
            [point[1] for point in collision_xy],
            color="#E53935",
            marker="x",
            s=36,
            label="碰撞",
            zorder=4,
        )
    bev_axis.set_xlabel("世界 X (m)")
    bev_axis.set_ylabel("世界 Y (m)")
    bev_axis.set_aspect("equal", adjustable="datalim")
    bev_axis.legend(frameon=False)
    bev_axis.grid(True, alpha=0.3)

    figure.suptitle(f"{scene_id} / {frames['strategy_name'].iloc[0]}")
    figure.tight_layout()
    path = save_fig(figure, run_dir, "ego_track")
    plt.close(figure)
    return path


def _collision_points(events, frames) -> list[tuple[float, float]]:
    if events.empty or "event_type" not in events.columns:
        return []
    hits = events.loc[events["event_type"] == "collision"]
    if hits.empty or "sim_time_s" not in hits.columns:
        return []
    points = []
    times = frames["sim_time_s"].to_numpy(dtype=float)
    xs = frames["ego_x_m"].to_numpy(dtype=float)
    ys = frames["ego_y_m"].to_numpy(dtype=float)
    for sim_time in hits["sim_time_s"].tolist():
        index = int(abs(times - float(sim_time)).argmin())
        points.append((float(xs[index]), float(ys[index])))
    return points
