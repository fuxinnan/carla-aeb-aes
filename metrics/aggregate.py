"""把多条指标收成 CSV 和没有合并单元格的 Markdown 表。"""

from __future__ import annotations

import csv
import statistics
from pathlib import Path

STRATEGIES = ("no_intervention", "fixed_ttc", "state_machine")
LABELS = {
    "no_intervention": "无干预",
    "fixed_ttc": "固定 TTC",
    "state_machine": "状态机",
}

EPISODE_FIELDS = (
    "scene_id",
    "strategy_name",
    "repeat_index",
    "seed",
    "hard_negative",
    "collided",
    "collision_relative_speed_mps",
    "min_distance_m",
    "false_brake",
    "activation_count",
    "exit_count",
    "state_switch_count",
    "risk_to_request_frames",
    "request_to_decel_frames",
    "peak_decel_mps2",
    "peak_jerk_mps3",
    "trigger_distance_m",
    "longitudinal_distance_m",
    "outcome",
)

SUMMARY_ROWS = (
    ("collision_rate", "碰撞率", "rate_collided"),
    ("collision_speed_mean_mps", "碰撞相对速度均值 (m/s)", "collision_speed"),
    ("min_distance_mean_m", "最小间距均值 (m)", "min_distance"),
    ("false_brake_rate", "硬负例误制动率", "false_brake"),
    ("activation_mean", "激活次数均值", "activation_count"),
    ("exit_mean", "退出次数均值", "exit_count"),
    ("state_switch_mean", "状态切换次数均值", "state_switch_count"),
    ("risk_to_request_mean_frames", "风险到请求延迟均值 (帧)", "risk_to_request_frames"),
    ("request_to_decel_mean_frames", "请求到实际减速延迟均值 (帧)", "request_to_decel_frames"),
    ("peak_decel_mean_mps2", "峰值减速度均值 (m/s²)", "peak_decel_mps2"),
    ("peak_jerk_mean_mps3", "加加速度峰值均值 (m/s³)", "peak_jerk_mps3"),
)


def write_episode_csv(rows: list[dict], path: str | Path) -> Path:
    """每次运行一行。重复跑出来的差别留在各行里，不先平均掉。"""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="utf-8", newline="\n") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(EPISODE_FIELDS), extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _cell(row.get(key)) for key in EPISODE_FIELDS})
    return target


def comparison_markdown(rows: list[dict]) -> str:
    """三列策略、一行一个指标。单元格不合并。"""
    header = "| 指标 | " + " | ".join(LABELS[name] for name in STRATEGIES) + " |"
    rule = "| --- | " + " | ".join("---" for _ in STRATEGIES) + " |"
    lines = [header, rule]
    for _key, label, kind in SUMMARY_ROWS:
        values = [_format_summary(rows, name, kind) for name in STRATEGIES]
        lines.append("| " + " | ".join([label, *values]) + " |")
    lines.append("")
    lines.append("| 场景 | 策略 | 重复 | 碰撞 | 碰撞相对速度 (m/s) | 最小间距 (m) | 误制动 | 风险到请求 (帧) | 请求到减速 (帧) |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for row in sorted(rows, key=_sort_key):
        lines.append(
            "| "
            + " | ".join(
                [
                    str(row["scene_id"]),
                    LABELS.get(row["strategy_name"], row["strategy_name"]),
                    _cell(row.get("repeat_index")),
                    "1" if row["collided"] else "0",
                    _plain(row.get("collision_relative_speed_mps")),
                    _plain(row.get("min_distance_m")),
                    _plain(row.get("false_brake")),
                    _plain(row.get("risk_to_request_frames")),
                    _plain(row.get("request_to_decel_frames")),
                ]
            )
            + " |"
        )
    lines.append("")
    lines.extend(stability_lines(rows))
    return "\n".join(lines) + "\n"


def write_comparison_markdown(rows: list[dict], path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(comparison_markdown(rows), encoding="utf-8")
    return target


def stability_lines(rows: list[dict]) -> list[str]:
    """同一场景、同一策略的重复是否一致。"""
    lines = ["| 场景 | 策略 | 重复次数 | 碰撞是否一致 | 相对速度极差 (m/s) | 误制动是否一致 |", "| --- | --- | --- | --- | --- | --- |"]
    grouped: dict[tuple, list[dict]] = {}
    for row in rows:
        grouped.setdefault((row["scene_id"], row["strategy_name"]), []).append(row)
    for key in sorted(grouped):
        group = grouped[key]
        collided = {bool(item["collided"]) for item in group}
        brakes = {item.get("false_brake") for item in group}
        speeds = [float(item["collision_relative_speed_mps"]) for item in group if item.get("collision_relative_speed_mps") is not None]
        spread = None if len(speeds) < 2 else max(speeds) - min(speeds)
        lines.append(
            "| "
            + " | ".join(
                [
                    key[0],
                    LABELS.get(key[1], key[1]),
                    str(len(group)),
                    "是" if len(collided) == 1 else "否",
                    _plain(spread),
                    "是" if len(brakes) == 1 else "否",
                ]
            )
            + " |"
        )
    return lines


def strategy_repeat_values(rows: list[dict], strategy: str, kind: str) -> list[float]:
    """每个重复先汇总成一个数，误差线用这些数的标准差。"""
    repeats = sorted({row.get("repeat_index") for row in rows})
    if repeats == [None]:
        repeats = [None]
    values = []
    for repeat in repeats:
        chosen = [row for row in rows if row["strategy_name"] == strategy and row.get("repeat_index") == repeat]
        value = _summary_value(chosen, kind)
        if value is not None:
            values.append(value)
    return values


def investigation_note(rows: list[dict]) -> str:
    """状态机碰撞多于固定 TTC 时，把对应场景和延迟写出来。不改指标。"""
    paired: dict[tuple, dict] = {}
    for row in rows:
        paired.setdefault((row["scene_id"], row.get("repeat_index")), {})[row["strategy_name"]] = row
    notes = []
    for key in sorted(paired, key=lambda item: (str(item[0]), -1 if item[1] is None else item[1])):
        group = paired[key]
        machine = group.get("state_machine")
        fixed = group.get("fixed_ttc")
        if machine is None or fixed is None:
            continue
        if machine["collided"] and not fixed["collided"]:
            notes.append(
                f"{key[0]} 重复 {_cell(key[1])}：固定 TTC 未碰撞，状态机碰撞，"
                f"相对速度 {_plain(machine.get('collision_relative_speed_mps'))} m/s。"
                f"风险到请求，状态机 {_plain(machine.get('risk_to_request_frames'))} 帧，"
                f"固定 TTC {_plain(fixed.get('risk_to_request_frames'))} 帧。"
            )
        if machine.get("false_brake") and not fixed.get("false_brake"):
            notes.append(
                f"{key[0]} 重复 {_cell(key[1])}：硬负例上状态机制动，固定 TTC 没有。"
                f"状态机风险到请求 {_plain(machine.get('risk_to_request_frames'))} 帧，"
                f"最小间距 {_plain(machine.get('min_distance_m'))} m。"
            )
    if not notes:
        return "这一批里，状态机的碰撞次数没有多于固定 TTC，硬负例上也没有多出的误制动。"
    return "下面这些差别先对照日志。本轮不改指标，也不改阈值。\n" + "\n".join(notes)


def _format_summary(rows: list[dict], strategy: str, kind: str) -> str:
    chosen = [row for row in rows if row["strategy_name"] == strategy]
    value = _summary_value(chosen, kind)
    spread = _repeat_spread(rows, strategy, kind)
    text = _plain(value)
    if spread is None:
        return text
    return f"{text}（重复极差 { _plain(spread) }）"


def _summary_value(rows: list[dict], kind: str) -> float | None:
    if kind == "rate_collided":
        if not rows:
            return None
        return sum(1.0 for row in rows if row["collided"]) / len(rows)
    if kind == "false_brake":
        cases = [row for row in rows if row["hard_negative"]]
        if not cases:
            return None
        return sum(1.0 for row in cases if row["false_brake"]) / len(cases)
    if kind == "collision_speed":
        speeds = [float(row["collision_relative_speed_mps"]) for row in rows if row["collided"] and row.get("collision_relative_speed_mps") is not None]
        if not speeds:
            return None
        return sum(speeds) / len(speeds)
    if kind == "min_distance":
        return _mean(row.get("min_distance_m") for row in rows)
    return _mean(row.get(kind) for row in rows)


def _repeat_spread(rows: list[dict], strategy: str, kind: str) -> float | None:
    values = strategy_repeat_values(rows, strategy, kind)
    if len(values) < 2:
        return None
    return max(values) - min(values)


def _mean(values) -> float | None:
    numbers = [float(value) for value in values if value is not None]
    if not numbers:
        return None
    return sum(numbers) / len(numbers)


def _sort_key(row: dict) -> tuple:
    repeat = row.get("repeat_index")
    return (str(row["scene_id"]), str(row["strategy_name"]), -1 if repeat is None else int(repeat))


def _cell(value) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _plain(value) -> str:
    if value is None:
        return "—"
    if isinstance(value, bool):
        return "是" if value else "否"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def sample_std(values: list[float]) -> float:
    """一个数时标准差按 0。没有数时调用方不要画误差线。"""
    if len(values) < 2:
        return 0.0
    return float(statistics.stdev(values))
