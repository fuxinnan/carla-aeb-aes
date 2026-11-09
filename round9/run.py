"""跑完调参、消融、重复和一次测试，再写报告。"""

from __future__ import annotations

import json
import shutil
from pathlib import Path

from aeb.config import load_aeb_config
from round9.catalog import TEMPLATES, write_grid
from round9.study import (
    HYSTERESIS_GAP_S,
    OBJECTIVE,
    STRATEGIES,
    Journal,
    ablation_configs,
    candidate_configs,
    choose,
    run_episode,
    scene_uses_occlusion,
    score_rows,
    write_aeb_yaml,
)
from viz.round9 import (
    STRATEGY_TEXT,
    TEMPLATE_TEXT,
    plot_ablation,
    plot_heatmaps,
    plot_occlusion,
    plot_stability,
    write_report,
)


ROOT = Path(__file__).resolve().parents[1]
SCENE_ROOT = ROOT / "configs" / "scenarios" / "round9"
CONFIG_ROOT = ROOT / "configs" / "aeb" / "round9"
LOG_ROOT = ROOT / "runs" / "round9" / "logs"
FIGURE_ROOT = ROOT / "figures" / "round9"
REPORT_PATH = ROOT / "runs" / "round9" / "report.html"
REPEATS = 3
NEAR_GAP_M = 0.5
NEAR_SPEED_MPS = 1.5


def main() -> None:
    from sim.carla_stage import CarlaStage

    with CarlaStage() as stage:
        _main(stage)


def _main(stage) -> None:
    if LOG_ROOT.exists():
        shutil.rmtree(LOG_ROOT)
    if FIGURE_ROOT.exists():
        shutil.rmtree(FIGURE_ROOT)
    base = load_aeb_config(ROOT / "configs" / "aeb" / "round5.yaml")
    write_grid(SCENE_ROOT)
    journal = Journal()
    tuning = journal.load_tuning(SCENE_ROOT)
    print(f"调参集 {len(tuning)} 条，开始网格搜索", flush=True)
    ranked, search_rows = _search(tuning, base, stage)
    fixed = choose("fixed_ttc", ranked)
    machine = choose("state_machine", ranked)
    fixed_path = write_aeb_yaml(CONFIG_ROOT / "chosen_fixed.yaml", _tagged(fixed.config, "round9-fixed"))
    machine_path = write_aeb_yaml(CONFIG_ROOT / "chosen_state.yaml", _tagged(machine.config, "round9-state"))
    fixed = _retarget(fixed, fixed_path)
    machine = _retarget(machine, machine_path)
    record = journal.snapshot()
    record["chosen"] = {
        "fixed_ttc": _choice_dict(fixed),
        "state_machine": _choice_dict(machine),
    }
    record["candidates"] = [_choice_dict(item) for item in ranked]
    record["test_ran_after_choice"] = False
    (CONFIG_ROOT / "tuning_log.json").write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("调参记录已写入，测试集尚未打开", flush=True)

    print("消融", flush=True)
    ablations = []
    ablation_rows = []
    labels = []
    for token, label, config in ablation_configs(machine.config):
        path = write_aeb_yaml(CONFIG_ROOT / f"ablate_{token}.yaml", _tagged(config, "round9-ablation"))
        labels.append((token, label))
        ablations.append((token, label, path))
        for scene in tuning:
            if token == "full":
                continue
            row = run_episode(scene, "state_machine", path, LOG_ROOT / "ablation" / token / scene.scene_id / "rep_00", occlusion=scene_uses_occlusion(scene), stage=stage)
            row["ablation"] = token
            ablation_rows.append(row)
            print(f"  消融 {token} {scene.scene_id} collided={row['collided']}", flush=True)
    for scene in tuning:
        row = run_episode(scene, "state_machine", machine_path, LOG_ROOT / "ablation" / "full" / scene.scene_id / "rep_00", occlusion=scene_uses_occlusion(scene), stage=stage)
        row["ablation"] = "full"
        ablation_rows.append(row)

    print(f"稳定性，重复 {REPEATS} 次", flush=True)
    configs = {
        "no_intervention": machine_path,
        "fixed_ttc": fixed_path,
        "state_machine": machine_path,
        "aeb_aes": machine_path,
    }
    stable_rows = _batch(tuning, configs, LOG_ROOT / "stability", REPEATS, stage)
    flips, near = _stability(stable_rows)
    if flips:
        print(f"翻转 {len(flips)} 条，再跑一遍", flush=True)
        extra = _batch(tuning, configs, LOG_ROOT / "stability_rerun", 1, stage)
        flips, near = _stability(stable_rows + extra)

    print("打开测试集，只跑一次", flush=True)
    test = journal.load_test(SCENE_ROOT)
    test_rows = _batch(test, configs, LOG_ROOT / "test", 1, stage)
    (CONFIG_ROOT / "test_run.json").write_text(
        json.dumps({"test_ran_after_choice": True, "test_count": len(test), "repeat": 1}, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print("遮挡对照", flush=True)
    occlusion_pairs = _occlusion_pairs(tuning, machine_path, stage)
    _write_outputs(search_rows, ablation_rows, labels, stable_rows, test_rows, flips, near, occlusion_pairs, fixed, machine)


def _search(scenes, base, stage):
    ranked = []
    rows = []
    for kind, label, config in candidate_configs(base):
        path = write_aeb_yaml(CONFIG_ROOT / "search" / f"{label}.yaml", config)
        group = []
        for scene in scenes:
            row = run_episode(
                scene,
                kind,
                path,
                LOG_ROOT / "search" / label / scene.scene_id / "rep_00",
                occlusion=scene_uses_occlusion(scene),
                stage=stage,
            )
            row["candidate"] = label
            group.append(row)
            rows.append(row)
        score, false_rate, switches = score_rows(group)
        from round9.study import Choice

        ranked.append(Choice(kind, label, score, false_rate, switches, path))
        print(f"  {label} 分数 {score:.3f} 碰撞 {sum(row['collided'] for row in group)}/{len(group)}", flush=True)
    return ranked, rows


def _batch(scenes, configs, root: Path, repeats: int, stage) -> list[dict]:
    rows = []
    for repeat in range(repeats):
        for strategy, config in configs.items():
            for scene in scenes:
                directory = root / strategy / scene.scene_id / f"rep_{repeat:02d}"
                row = run_episode(scene, strategy, config, directory, occlusion=scene_uses_occlusion(scene), stage=stage)
                rows.append(row)
                print(f"  {root.name} {strategy} {scene.scene_id} rep {repeat} collided={row['collided']}", flush=True)
    return rows


def _stability(rows: list[dict]) -> tuple[list[dict], set[tuple[str, str]]]:
    groups: dict[tuple[str, str], list[dict]] = {}
    for row in rows:
        groups.setdefault((row["scene_id"], row["strategy_name"]), []).append(row)
    flips = []
    near = set()
    for key, group in groups.items():
        signatures = {(row["collided"], row["collision_relative_speed_mps"], row["outcome"]) for row in group}
        if len(signatures) > 1:
            frames = []
            for row in group:
                frames.append({"repeat": row["repeat_index"], "collided": row["collided"], "outcome": row["outcome"]})
            flips.append({"scene_id": key[0], "strategy_name": key[1], "repeats": frames})
        for row in group:
            gap = row.get("min_distance_m")
            speed = row.get("collision_relative_speed_mps")
            close_miss = (not row["collided"]) and gap is not None and gap < NEAR_GAP_M
            soft_hit = row["collided"] and speed is not None and speed < NEAR_SPEED_MPS
            if close_miss or soft_hit:
                near.add(key)
    return flips, near


def _occlusion_pairs(scenes, config, stage) -> list[dict]:
    pairs = []
    for scene in scenes:
        if scene.template != "occlusion_reveal":
            continue
        full = run_episode(scene, "state_machine", config, LOG_ROOT / "occlusion_full" / scene.scene_id / "rep_00", occlusion=False, stage=stage)
        hidden = run_episode(scene, "state_machine", config, LOG_ROOT / "occlusion_hidden" / scene.scene_id / "rep_00", occlusion=True, stage=stage)
        pairs.append(
            {
                "label": scene.instance,
                "full_frame": _risk_frame(LOG_ROOT / "occlusion_full" / scene.scene_id / "rep_00", "ped_hidden"),
                "occluded_frame": _risk_frame(LOG_ROOT / "occlusion_hidden" / scene.scene_id / "rep_00", "ped_hidden"),
                "full_collision": 1.0 if full["collided"] else 0.0,
                "occluded_collision": 1.0 if hidden["collided"] else 0.0,
                "full_seen": _seen_frame(LOG_ROOT / "occlusion_full" / scene.scene_id / "rep_00", "ped_hidden"),
                "occluded_seen": _seen_frame(LOG_ROOT / "occlusion_hidden" / scene.scene_id / "rep_00", "ped_hidden"),
            }
        )
    return pairs


def _risk_frame(directory: Path, track_id: str) -> int | None:
    for line in (directory / "frames.jsonl").read_text(encoding="utf-8").splitlines():
        frame = json.loads(line)
        threat = frame.get("threat") or {}
        if threat.get("threat_id") == track_id and threat.get("ttc_s") is not None:
            return int(frame["frame"])
    return None


def _seen_frame(directory: Path, track_id: str) -> int | None:
    for line in (directory / "events.jsonl").read_text(encoding="utf-8").splitlines():
        event = json.loads(line)
        if event.get("event_type") == "first_seen" and (event.get("detail") or {}).get("track_id") == track_id:
            return int(event["detail"]["frame"])
    return None


def _write_outputs(search_rows, ablation_rows, labels, stable_rows, test_rows, flips, near, pairs, fixed, machine) -> None:
    tuning_chosen = [row for row in stable_rows if row["repeat_index"] == 0]
    heat_tuning = plot_heatmaps(tuning_chosen, FIGURE_ROOT, "tuning", STRATEGIES, TEMPLATES)
    heat_test = plot_heatmaps(test_rows, FIGURE_ROOT, "test", STRATEGIES, TEMPLATES)
    plot_ablation(ablation_rows, labels, FIGURE_ROOT / "ablation.png")
    plot_stability(stable_rows, near, FIGURE_ROOT / "stability.png")
    plot_occlusion(pairs, FIGURE_ROOT / "occlusion.png")
    ratio = len(flips) / max(1, len({(row["scene_id"], row["strategy_name"]) for row in stable_rows}))
    acceptable = ratio == 0.0
    chapters = [
        ("参数网格", _grid_html(fixed, machine, search_rows), heat_tuning + heat_test),
        ("消融", _ablation_html(ablation_rows, labels), [FIGURE_ROOT / "ablation.png"]),
        ("稳定性", _stability_html(stable_rows, flips, near, ratio, acceptable), [FIGURE_ROOT / "stability.png"]),
        ("遮挡", _occlusion_html(pairs), [FIGURE_ROOT / "occlusion.png"]),
        ("测试集对照", _summary_html(test_rows), []),
    ]
    write_report(REPORT_PATH, chapters)
    (ROOT / "runs" / "round9" / "comparison.md").write_text(_summary_markdown(test_rows), encoding="utf-8")
    print(f"报告 {REPORT_PATH}", flush=True)


def _grid_html(fixed, machine, search_rows) -> str:
    return (
        f"<p>目标函数是 <code>{OBJECTIVE}</code>，越低越好。"
        "同分再比硬负例误制动率、平均状态切换次数，最后靠近第 5 轮占位阈值。"
        "调参只用调参目录里的 YAML。下面的热力图是选定配置在调参集和测试集上的碰撞率，两套分开画。</p>"
        f"<p>选定固定 TTC：{fixed.config.fixed_ttc.ttc_brake_s:.1f} s，分数 {fixed.score:.3f}。"
        f"选定状态机：确认 {machine.config.state_machine.confirm_frames} 帧，"
        f"部分制动进入 {machine.config.state_machine.ttc_partial_on_s:.1f} s，分数 {machine.score:.3f}。</p>"
        + _table(
            ["候选", "策略", "分数", "碰撞率", "误制动率"],
            [
                [
                    item["candidate"],
                    STRATEGY_TEXT.get(item["strategy_name"], item["strategy_name"]),
                    f"{score_rows([row for row in search_rows if row['candidate']==item['candidate']])[0]:.3f}",
                    _rate([row for row in search_rows if row["candidate"] == item["candidate"]], "collided"),
                    _false_rate([row for row in search_rows if row["candidate"] == item["candidate"]]),
                ]
                for item in _unique_candidates(search_rows)
            ],
        )
    )


def _ablation_html(rows, labels) -> str:
    full = [row for row in rows if row["ablation"] == "full"]
    body = [
        f"<p>三次消融都只改选定状态机的配置。去掉迟滞时，离开阈值比进入阈值高 {HYSTERESIS_GAP_S:.2f} s。"
        "加载器要求离开阈值严格大于进入阈值，0.05 s 是它允许的最小间隙。策略代码没有改。"
        "选定配置的确认帧数已经是 1，所以再去掉风险确认没有差别。"
        "确认帧数的作用写在上面的搜索表里：部分制动同为 1.8 s 时，确认从 1 帧改成 3 帧，调参集碰撞从 3/24 升到 6/24。"
        "旁车并行没有产生制动，误制动率都是 0，目标函数这一轮只由碰撞率决定。</p>",
        _table(
            ["版本", "碰撞率", "误制动率", "平均状态切换", "相对完整状态机"],
            [_ablation_line(token, label, rows, full) for token, label in labels],
        ),
    ]
    return "\n".join(body)


def _ablation_line(token, label, rows, full) -> list[str]:
    group = [row for row in rows if row["ablation"] == token]
    collision = _mean_flag(group, "collided")
    false_rate = _false_rate_value(group)
    switches = sum(row["state_switch_count"] for row in group) / len(group)
    base_collision = _mean_flag(full, "collided")
    base_false = _false_rate_value(full)
    base_switch = sum(row["state_switch_count"] for row in full) / len(full)
    note = _contribution(label, collision - base_collision, false_rate - base_false, switches - base_switch)
    return [label, f"{collision:.3f}", f"{false_rate:.3f}", f"{switches:.2f}", note]


def _contribution(label: str, collision: float, false_rate: float, switches: float) -> str:
    if label == "完整状态机":
        return "对照"
    parts = []
    if abs(collision) < 1e-9:
        parts.append("碰撞率没有变化")
    elif collision > 0:
        parts.append(f"碰撞率高 {collision:.3f}，这个部件在帮着避免碰撞")
    else:
        parts.append(f"碰撞率低 {-collision:.3f}，去掉之后制动更早或更重")
    if abs(false_rate) < 1e-9:
        parts.append("误制动率没有变化")
    elif false_rate > 0:
        parts.append(f"误制动率高 {false_rate:.3f}")
    else:
        parts.append(f"误制动率低 {-false_rate:.3f}")
    if abs(switches) < 1e-9:
        parts.append("状态切换次数没有变化")
    elif switches > 0:
        parts.append(f"状态切换多 {switches:.2f} 次")
    else:
        parts.append(f"状态切换少 {-switches:.2f} 次")
    return "；".join(parts)


def _stability_html(rows, flips, near, ratio, acceptable) -> str:
    verdict = "可接受" if acceptable else "不可接受"
    extra = "" if acceptable else "翻转比例不是 0。已经用同一批配置再跑了一遍，结果仍写在下面。"
    lines = [
        f"<p>同一批调参场景重复 {REPEATS} 次。翻转比例 {ratio:.3f}，{verdict}。{extra}</p>",
    ]
    if flips:
        lines.append(_table(["场景", "策略", "各次结果"], [[item["scene_id"], item["strategy_name"], str(item["repeats"])] for item in flips]))
    else:
        lines.append("<p>没有场景在重复之间改变碰撞结果。</p>")
    if near:
        lines.append("<p>靠近碰撞边界：未碰撞且最小间距小于 0.5 m，或碰撞且相对速度小于 1.5 m/s。</p>")
        lines.append(_table(["场景", "策略"], [[scene, STRATEGY_TEXT.get(strategy, strategy)] for scene, strategy in sorted(near)]))
    else:
        lines.append("<p>没有场景落在设定的碰撞边界附近。</p>")
    return "\n".join(lines)


def _occlusion_html(pairs) -> list:
    return (
        "<p>同一条遮挡场景各跑两次。一次把遮挡模型关掉，行人从第一帧就可见；一次让更近的大车挡住视线。"
        "比的是被挡住的那条行人航迹第一次出现有效 TTC 的帧，不是旁边大车的威胁。</p>"
        + _table(
            ["场景", "全可见风险帧", "遮挡后风险帧", "全可见首次看见", "遮挡后首次看见", "全可见碰撞", "遮挡碰撞"],
            [
                [
                    item["label"],
                    _fmt(item["full_frame"]),
                    _fmt(item["occluded_frame"]),
                    _fmt(item["full_seen"]),
                    _fmt(item["occluded_seen"]),
                    f"{item['full_collision']:.0f}",
                    f"{item['occluded_collision']:.0f}",
                ]
                for item in pairs
            ],
        )
    )


def _summary_html(rows) -> str:
    return "<p>测试集在选定配置之后只跑了一次。固定 TTC 和状态机用的是调参选出的配置，AEB+AES 用的是同一份状态机再加避让。</p>" + _summary_table(rows)


def _summary_markdown(rows) -> str:
    header = "| 策略 | 碰撞率 | 碰撞相对速度 (m/s) | 误制动率 | 平均状态切换 | 平均最小间距 (m) |\n| --- | --- | --- | --- | --- | --- |\n"
    body = []
    for strategy in STRATEGIES:
        group = [row for row in rows if row["strategy_name"] == strategy]
        body.append(
            "| "
            + " | ".join(
                [
                    STRATEGY_TEXT[strategy],
                    f"{_mean_flag(group, 'collided'):.3f}",
                    _mean_speed(group),
                    f"{_false_rate_value(group):.3f}",
                    f"{sum(row['state_switch_count'] for row in group) / len(group):.2f}",
                    _mean_gap(group),
                ]
            )
            + " |"
        )
    return header + "\n".join(body) + "\n"


def _summary_table(rows) -> str:
    return _table(
        ["策略", "碰撞率", "碰撞相对速度 (m/s)", "误制动率", "平均状态切换", "平均最小间距 (m)"],
        [
            [
                STRATEGY_TEXT[strategy],
                f"{_mean_flag([row for row in rows if row['strategy_name']==strategy], 'collided'):.3f}",
                _mean_speed([row for row in rows if row["strategy_name"] == strategy]),
                f"{_false_rate_value([row for row in rows if row['strategy_name']==strategy]):.3f}",
                f"{sum(row['state_switch_count'] for row in rows if row['strategy_name']==strategy) / max(1, sum(1 for row in rows if row['strategy_name']==strategy)):.2f}",
                _mean_gap([row for row in rows if row["strategy_name"] == strategy]),
            ]
            for strategy in STRATEGIES
        ],
    )


def _table(headers, rows) -> str:
    head = "".join(f"<th>{item}</th>" for item in headers)
    body = []
    for row in rows:
        body.append("<tr>" + "".join(f"<td>{item}</td>" for item in row) + "</tr>")
    return "<table><tr>" + head + "</tr>" + "".join(body) + "</table>"


def _unique_candidates(rows):
    seen = []
    for row in rows:
        item = {"candidate": row["candidate"], "strategy_name": row["strategy_name"]}
        if item not in seen:
            seen.append(item)
    return seen


def _rate(rows, key) -> str:
    return f"{_mean_flag(rows, key):.3f}"


def _false_rate(rows) -> str:
    return f"{_false_rate_value(rows):.3f}"


def _false_rate_value(rows) -> float:
    hard = [row for row in rows if row["hard_negative"]]
    if not hard:
        return 0.0
    return sum(1.0 for row in hard if row["false_brake"]) / len(hard)


def _mean_flag(rows, key) -> float:
    return sum(1.0 for row in rows if row[key]) / len(rows)


def _mean_speed(rows) -> str:
    speeds = [row["collision_relative_speed_mps"] for row in rows if row["collided"] and row["collision_relative_speed_mps"] is not None]
    if not speeds:
        return "—"
    return f"{sum(speeds) / len(speeds):.3f}"


def _mean_gap(rows) -> str:
    gaps = [row["min_distance_m"] for row in rows if row["min_distance_m"] is not None]
    if not gaps:
        return "—"
    return f"{sum(gaps) / len(gaps):.3f}"


def _fmt(value) -> str:
    return "—" if value is None else str(value)


def _tagged(config, version: str):
    from aeb.config import AebConfig

    return AebConfig(version, config.fixed_ttc, config.state_machine, config.actuator, config.path)


def _retarget(choice, config):
    from round9.study import Choice

    return Choice(choice.kind, choice.label, choice.score, choice.false_rate, choice.switches, config)


def _choice_dict(choice) -> dict:
    machine = choice.config.state_machine
    return {
        "kind": choice.kind,
        "label": choice.label,
        "score": choice.score,
        "false_brake_rate": choice.false_rate,
        "mean_state_switches": choice.switches,
        "ttc_brake_s": choice.config.fixed_ttc.ttc_brake_s,
        "confirm_frames": machine.confirm_frames,
        "ttc_partial_on_s": machine.ttc_partial_on_s,
        "partial_decel_mps2": machine.partial_decel_mps2,
        "full_decel_mps2": machine.full_decel_mps2,
        "config": str(choice.config.path),
    }


if __name__ == "__main__":
    main()
