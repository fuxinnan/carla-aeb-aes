"""第 8 轮的跟踪误差、附着圆、仲裁色带和四策略俯视对比。"""

from __future__ import annotations

from pathlib import Path

from arbiter.coordinator import reason_text
from control.round8 import load_round8_config
from viz.common import STRATEGY_COLORS, _prepare_matplotlib


_REASON_COLORS = {
    "aeb_only_no_conflict": "#B0BEC5",
    "aeb_only_brake_sufficient": "#1565C0",
    "aes_selected": "#43A047",
    "aes_tracking": "#2E7D32",
    "aes_infeasible": "#FB8C00",
    "aes_completed": "#1E88E5",
    "friction_limited": "#6A1B9A",
}


def plot_tracking(result, path: str | Path) -> Path:
    _prepare_matplotlib()
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 1, figsize=(8.5, 7.2))
    reference_x = [sample.ref_x_m for sample in result.samples]
    reference_y = [sample.ref_y_m for sample in result.samples]
    actual_x = [sample.x_m for sample in result.samples]
    actual_y = [sample.y_m for sample in result.samples]
    axes[0].plot(reference_x, reference_y, color="#546E7A", linewidth=2.0, label="参考轨迹")
    axes[0].plot(actual_x, actual_y, color="#1565C0", linewidth=1.6, label="实际轨迹")
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].set_xlabel("世界 X (m)")
    axes[0].set_ylabel("世界 Y (m)")
    axes[0].set_title("纯追踪：参考圆弧和实际轨迹")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(frameon=False)
    times = [sample.t_s for sample in result.samples]
    errors = [sample.error_m for sample in result.samples]
    axes[1].plot(times, errors, color="#1565C0", linewidth=1.6)
    axes[1].axhline(result.threshold_m, color="#E53935", linestyle="--", linewidth=1.0, label=f"门槛 ±{result.threshold_m:.2f} m")
    axes[1].axhline(-result.threshold_m, color="#E53935", linestyle="--", linewidth=1.0)
    axes[1].set_xlabel("时间 (s)")
    axes[1].set_ylabel("横向误差 (m)")
    axes[1].set_title(f"最大绝对值 {result.max_abs_error_m:.3f} m")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(frameon=False)
    figure.tight_layout()
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=120)
    plt.close(figure)
    return output


def plot_friction(records, path: str | Path, mu: float | None = None, g_mps2: float | None = None) -> Path:
    _prepare_matplotlib()
    import matplotlib.pyplot as plt

    config = load_round8_config()
    used_mu = config.friction.mu if mu is None else mu
    used_g = config.friction.g_mps2 if g_mps2 is None else g_mps2
    radius = used_mu * used_g
    figure, axis = plt.subplots(figsize=(6.4, 6.2))
    long = [record.ego.a_long_mps2 for record in records]
    lat = [record.ego.a_lat_mps2 for record in records]
    axis.scatter(long, lat, s=18, color="#6A1B9A", alpha=0.8, label="每帧实测")
    circle = plt.Circle((0.0, 0.0), radius, fill=False, linestyle="--", color="#212121", linewidth=1.2, label=f"附着圆 {radius:.2f} m/s²")
    axis.add_patch(circle)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("纵向加速度 (m/s²)，制动为负")
    axis.set_ylabel("横向加速度 (m/s²)，向右为正")
    axis.set_title("附着圆")
    axis.grid(True, alpha=0.3)
    axis.legend(frameon=False, loc="upper right")
    limit = radius * 1.15
    axis.set_xlim(-limit, limit)
    axis.set_ylim(-limit, limit)
    figure.tight_layout()
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=120)
    plt.close(figure)
    return output


def plot_arbitration(records, path: str | Path) -> Path:
    _prepare_matplotlib()
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    figure, axis = plt.subplots(figsize=(10.0, 3.6))
    if records:
        width = records[1].sim_time_s - records[0].sim_time_s if len(records) > 1 else 0.05
    else:
        width = 0.05
    seen = []
    for record in records:
        reason = record.arbitration_reason or "aeb_only_no_conflict"
        key = "friction_limited" if "friction_limited" in reason else reason.split("|", 1)[0]
        color = _REASON_COLORS.get(key, "#212121")
        axis.add_patch(Rectangle((record.sim_time_s, 0.0), width, 1.0, facecolor=color, edgecolor="none"))
        if not seen or seen[-1][0] != reason:
            seen.append((reason, record.sim_time_s))
    notes = []
    for index, (reason, time) in enumerate(seen):
        axis.axvline(time, color="#212121", linewidth=0.6, alpha=0.7)
        notes.append(f"{time:.2f} s  {reason_text(reason)}")
        axis.text(time, 1.08 + 0.22 * (index % 2), f"{time:.2f}s", fontsize=8, ha="left", va="bottom")
    axis.set_ylim(0.0, 1.7)
    axis.set_yticks([])
    axis.set_xlabel("时间 (s)")
    axis.set_title("仲裁色带。灰=无冲突，蓝=只用制动，绿=避让，橙=避让不可行，紫=附着圆限制")
    if notes:
        axis.text(
            0.0,
            -0.42,
            "切换：" + "；".join(notes),
            transform=axis.transAxes,
            fontsize=8,
            va="top",
        )
    figure.tight_layout()
    figure.subplots_adjust(bottom=0.28)
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=120)
    plt.close(figure)
    return output


def plot_strategy_bev(named_episodes, path: str | Path) -> Path:
    """named_episodes 是 (策略名, ClosedEpisode) 。碰撞画叉，侧向间距最小的帧画方块。"""
    _prepare_matplotlib()
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 1, figsize=(11.0, 7.4), gridspec_kw={"height_ratios": [1.15, 1.0]})
    bev, decel = axes
    longest = max(named_episodes, key=lambda item: len(item[1].steps))[1] if named_episodes else None
    if longest is not None:
        target_x = [step.target.x_m for step in longest.steps]
        target_y = [step.target.y_m for step in longest.steps]
        bev.plot(target_x, target_y, color="#90A4AE", linewidth=1.2, linestyle="--", label="行人")
    bev.axhline(0.0, color="#CFD8DC", linewidth=1.0)
    bev.axhline(1.75, color="#CFD8DC", linewidth=0.8, linestyle=":")
    bev.axhline(-1.75, color="#CFD8DC", linewidth=0.8, linestyle=":")
    labels = []
    handles = []
    for name, episode in named_episodes:
        color = STRATEGY_COLORS.get(name, "#212121")
        xs = [step.ego.x_m for step in episode.steps]
        ys = [step.ego.y_m for step in episode.steps]
        line, = bev.plot(xs, ys, color=color, linewidth=1.8, label=_label(name, episode))
        handles.append(line)
        labels.append(_label(name, episode))
        if episode.collided:
            hit = episode.steps[-1]
            bev.scatter([hit.ego.x_m], [hit.ego.y_m], marker="X", s=70, color=color, zorder=4)
        risk = _min_side_step(episode)
        if risk is not None:
            bev.scatter([risk.ego.x_m], [risk.ego.y_m], marker="s", s=36, color=color, zorder=3)
        times = [step.sim_time_s for step in episode.steps]
        values = [max(0.0, -step.record.ego.a_long_mps2) for step in episode.steps]
        decel.plot(times, values, color=color, linewidth=1.6)
    bev.set_aspect("equal", adjustable="box")
    bev.set_xlabel("世界 X (m)")
    bev.set_ylabel("世界 Y (m)，向右为正")
    bev.set_title("俯视轨迹。叉是碰撞，方块是侧向间距最小的帧，虚线是行人")
    bev.grid(True, alpha=0.3)
    decel.set_xlabel("时间 (s)")
    decel.set_ylabel("减速度 (m/s²)")
    decel.set_title("四条策略的实测减速度")
    decel.grid(True, alpha=0.3)
    figure.legend(handles, labels, loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 1.02))
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.90))
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=120)
    plt.close(figure)
    return output


def _min_side_step(episode):
    chosen = None
    best = None
    for step in episode.steps:
        gap = step.record.side_gap_m
        if gap is None:
            continue
        if best is None or gap < best:
            best = gap
            chosen = step
    return chosen


def _label(name: str, episode) -> str:
    names = {
        "no_intervention": "无干预",
        "fixed_ttc": "固定 TTC",
        "state_machine": "状态机 AEB",
        "aeb_aes": "状态机 AEB+AES",
    }
    outcome = "碰撞" if episode.collided else "未碰撞"
    speed = "—" if episode.collision_relative_speed_mps is None else f"{episode.collision_relative_speed_mps:.2f} m/s"
    return f"{names.get(name, name)} {outcome} {speed}"
