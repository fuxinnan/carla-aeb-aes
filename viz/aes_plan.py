"""AES 候选轨迹、拒绝原因和动力学曲线。不连接仿真器。"""

from __future__ import annotations

import textwrap
from dataclasses import replace
from pathlib import Path

from scenarios.geometry import box_corners, forward_xy

from aes.cases import build_cases
from aes.config import AesConfig, load_aes_config
from aes.planner import plan_avoidance
from aes.trajectory import corridor_edges, world_velocity
from aes.types import DIRECTION_LEFT, DIRECTION_RIGHT, AesPlan, direction_text
from viz.common import _prepare_matplotlib


LEFT_COLOR = "#1565C0"
RIGHT_COLOR = "#C62828"
TARGET_LIGHT = (1.0, 0.80, 0.82)
TARGET_DARK = (0.72, 0.11, 0.11)


def render_aes_figures(directory: str | Path, config: AesConfig | None = None) -> tuple[Path, Path]:
    """写出四格总览和动力学曲线。总览文件名是 aes_cases.png。"""
    _prepare_matplotlib()
    loaded = config if config is not None else load_aes_config()
    cases = build_cases()
    plans = tuple(plan_avoidance(case.ego, case.targets, case.road, case.corridor, loaded) for case in cases)
    output = Path(directory)
    output.mkdir(parents=True, exist_ok=True)
    overview = output / "aes_cases.png"
    dynamics = output / "aes_dynamics.png"
    _draw_overview(cases, plans, loaded, overview)
    _draw_dynamics(cases, plans, loaded, dynamics)
    return overview, dynamics


def _draw_overview(cases, plans, config: AesConfig, path: Path) -> None:
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 2, figsize=(16.0, 9.0))
    for axis, case, plan in zip(axes.ravel(), cases, plans):
        _draw_scene(axis, case, plan)
    limits = config.constraints
    figure.suptitle("AES 候选轨迹与可行性", fontsize=14)
    figure.text(
        0.01,
        0.01,
        "\n".join(
            [
                f"可行驶边界：{plans[0].boundary_source}",
                (
                    f"限值：横向加速度 ±{limits.max_lat_accel_mps2:.1f} m/s²，"
                    f"前轮转角 ±{limits.max_steer_rad:.2f} rad，"
                    f"转角变化率 ±{limits.max_steer_rate_radps:.2f} rad/s，"
                    f"轴距 {limits.wheelbase_m:.2f} m"
                ),
            ]
        ),
        fontsize=8,
        va="bottom",
    )
    figure.tight_layout(rect=(0.0, 0.06, 1.0, 0.96))
    figure.savefig(path, dpi=120)
    plt.close(figure)


def _draw_scene(axis, case, plan: AesPlan) -> None:
    from matplotlib.patches import Polygon

    left_edge, right_edge = corridor_edges(case.road, case.corridor)
    polygon = left_edge + list(reversed(right_edge))
    axis.add_patch(Polygon(polygon, closed=True, facecolor="#ECEFF1", edgecolor="none", zorder=0))
    axis.plot([point[0] for point in left_edge], [point[1] for point in left_edge], color="#546E7A", linewidth=1.0)
    axis.plot([point[0] for point in right_edge], [point[1] for point in right_edge], color="#546E7A", linewidth=1.0, label="可行驶边界")
    axis.plot([sample.x_m for sample in case.road], [sample.y_m for sample in case.road], color="#90A4AE", linewidth=0.8, linestyle=":")
    horizon = max(candidate.points[-1].t_s for candidate in plan.candidates)
    _draw_targets(axis, case.targets, horizon)
    for candidate in plan.candidates:
        color = LEFT_COLOR if candidate.direction == DIRECTION_LEFT else RIGHT_COLOR
        style = "-" if candidate.feasible else "--"
        label = f"{direction_text(candidate.direction)}{'可行' if candidate.feasible else '不可行'}"
        axis.plot(
            [point.x_m for point in candidate.points],
            [point.y_m for point in candidate.points],
            color=color,
            linestyle=style,
            linewidth=2.4,
            label=label,
            zorder=3,
        )
        _draw_body_at(axis, candidate, case.ego, color)
    _draw_ego_now(axis, case.ego)
    axis.set_xlim(0.0, 52.0)
    axis.set_ylim(-8.0, 8.0)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("世界 X (m)")
    axis.set_ylabel("世界 Y (m)")
    axis.set_title(case.title)
    axis.grid(True, alpha=0.25)
    axis.legend(loc="lower right", frameon=False, fontsize=8)
    axis.text(
        0.02,
        0.98,
        _decision_text(plan),
        transform=axis.transAxes,
        va="top",
        ha="left",
        fontsize=8,
        bbox={"boxstyle": "round", "facecolor": "white", "edgecolor": "#BDBDBD", "alpha": 0.94},
    )


def _draw_targets(axis, targets, horizon_s: float) -> None:
    from matplotlib.patches import Polygon

    times = _fade_times(horizon_s)
    for target in targets:
        vx, vy = world_velocity(target)
        fx, fy = forward_xy(target.yaw_rad)
        for index, time in enumerate(times):
            weight = index / max(1, len(times) - 1)
            color = tuple(light + (dark - light) * weight for light, dark in zip(TARGET_LIGHT, TARGET_DARK))
            corners = box_corners(
                target.x_m + vx * time,
                target.y_m + vy * time,
                target.yaw_rad,
                target.length_m,
                target.width_m,
            )
            axis.add_patch(
                Polygon(corners, closed=True, facecolor=color, edgecolor=color, alpha=0.28 + 0.45 * weight, linewidth=0.8, zorder=2)
            )
        axis.annotate(
            "",
            xy=(target.x_m + fx * 1.6, target.y_m + fy * 1.6),
            xytext=(target.x_m, target.y_m),
            arrowprops={"arrowstyle": "->", "color": "#B71C1C", "lw": 1.2},
        )


def _draw_body_at(axis, candidate, ego, color: str) -> None:
    from matplotlib.patches import Polygon

    if candidate.first_overlap_time_s is None:
        point = candidate.points[-1]
    else:
        point = min(candidate.points, key=lambda item: abs(item.t_s - candidate.first_overlap_time_s))
    corners = box_corners(point.x_m, point.y_m, point.yaw_rad, ego.length_m, ego.width_m)
    axis.add_patch(Polygon(corners, closed=True, facecolor="none", edgecolor=color, linewidth=1.2, linestyle=":", zorder=4))


def _draw_ego_now(axis, ego) -> None:
    from matplotlib.patches import Polygon

    corners = box_corners(ego.x_m, ego.y_m, ego.yaw_rad, ego.length_m, ego.width_m)
    axis.add_patch(Polygon(corners, closed=True, facecolor="#1565C0", edgecolor="#1565C0", alpha=0.85, linewidth=1.0, zorder=5, label="自车"))


def _draw_dynamics(cases, plans, config: AesConfig, path: Path) -> None:
    import matplotlib.pyplot as plt

    short = _short_plan(cases[0], config)
    rows = list(zip((case.title for case in cases), plans))
    rows.append(("对照：纵向缩短到 10 m", short))
    figure, axes = plt.subplots(len(rows), 3, figsize=(14.0, 16.0), sharex=True)
    limits = config.constraints
    for row, (title, plan) in enumerate(rows):
        _series(axes[row, 0], plan, "a_lat_mps2", "横向加速度 (m/s²)", limits.max_lat_accel_mps2)
        _series(axes[row, 1], plan, "steer_rad", "前轮转角 (rad)", limits.max_steer_rad)
        _series(axes[row, 2], plan, "steer_rate_radps", "前轮转角变化率 (rad/s)", limits.max_steer_rate_radps)
        axes[row, 0].set_title(title, loc="left", fontsize=10)
    for column in range(3):
        axes[-1, column].set_xlabel("时间 (s)")
    figure.suptitle("候选轨迹的横向加速度、前轮转角和转角变化率", fontsize=14)
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
    figure.savefig(path, dpi=120)
    plt.close(figure)


def _series(axis, plan: AesPlan, field: str, ylabel: str, limit: float) -> None:
    for candidate in plan.candidates:
        color = LEFT_COLOR if candidate.direction == DIRECTION_LEFT else RIGHT_COLOR
        style = "-" if candidate.feasible else "--"
        axis.plot(
            [point.t_s for point in candidate.points],
            [getattr(point, field) for point in candidate.points],
            color=color,
            linestyle=style,
            linewidth=1.6,
            label=direction_text(candidate.direction),
        )
    axis.axhline(limit, color="#212121", linewidth=1.0, linestyle=":", label=f"限值 ±{limit:.2f}")
    axis.axhline(-limit, color="#212121", linewidth=1.0, linestyle=":")
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.25)
    axis.legend(loc="upper right", frameon=False, fontsize=8)


def _short_plan(case, config: AesConfig) -> AesPlan:
    """同样的横向偏移，纵向只留 10 m。用来确认限值线会挡住过急的轨迹。"""
    shortened = replace(config, trajectory=replace(config.trajectory, length_m=10.0))
    return plan_avoidance(case.ego, (), case.road, case.corridor, shortened)


def _fade_times(horizon_s: float) -> list[float]:
    step = 0.25
    times = [0.0]
    time = step
    while time < horizon_s - 1e-9:
        times.append(time)
        time += step
    if horizon_s - times[-1] > 1e-6:
        times.append(horizon_s)
    return times


def _decision_text(plan: AesPlan) -> str:
    if plan.avoidable:
        lines = [f"选用{direction_text(plan.direction)}"]
    else:
        lines = ["两侧都拒绝，不硬选"]
    for rejection in plan.rejections:
        detail = "\n".join(textwrap.wrap(rejection.text, width=22))
        lines.append(f"{direction_text(rejection.direction)}：{detail}")
    return "\n".join(lines)
