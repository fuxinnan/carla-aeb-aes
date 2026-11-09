"""挖难例、在调参侧加密、回放选中的案例，并跑回归。"""

from __future__ import annotations

import json
from pathlib import Path

from round10.guard import ROOT, refuse_test_write
from round10.mine import CAUSE_TEXT, inspect_run, iter_run_dirs, load_thresholds
from round10.neighborhood import write_neighborhood
from round10.regress import run_regression
from round10.replay import replay
from viz.round10 import export_replay, plot_causes, plot_neighborhood, plot_regression, write_report


CASE_DIR = ROOT / "regression" / "cases"
NEIGHBOR_DIR = ROOT / "configs" / "scenarios" / "round9" / "neighborhood" / "tuning"
FIGURE_DIR = ROOT / "figures" / "round10"
REPORT_PATH = ROOT / "runs" / "round10" / "report.html"


def main() -> None:
    from sim.carla_stage import CarlaStage

    with CarlaStage() as stage:
        _main(stage)


def _main(stage) -> None:
    thresholds = load_thresholds()
    runs = iter_run_dirs(thresholds["log_roots"])
    cases = []
    for index, run_dir in enumerate(runs, start=1):
        found = inspect_run(run_dir, thresholds)
        if found is not None:
            cases.append(found)
        if index % 40 == 0 or index == len(runs):
            print(f"已读 {index}/{len(runs)}，难例 {len(cases)}", flush=True)
    selected = _select(cases)
    _write_cases(selected)
    generated = write_neighborhood(selected, NEIGHBOR_DIR)
    rows = run_regression(stage=stage)
    _check_late_brake(selected, rows)
    plot_causes(cases, FIGURE_DIR / "causes.png")
    plot_neighborhood(selected, generated, FIGURE_DIR / "neighborhood.png")
    plot_regression(rows, FIGURE_DIR / "regression.png")
    stills = []
    for case in selected:
        if case["suspected_cause"] != "late_brake":
            continue
        result = replay(case["manifest"], stage)
        exported = export_replay(result, case["case_id"], FIGURE_DIR / "replay" / case["case_id"])
        stills.extend(path for name, path in exported.items() if name != "bev")
    write_report(
        REPORT_PATH,
        [
            ("难例线索", _cause_html(cases), [FIGURE_DIR / "causes.png"]),
            ("邻域", _neighbor_html(generated), [FIGURE_DIR / "neighborhood.png"]),
            ("回归", _regress_html(rows), [FIGURE_DIR / "regression.png"]),
            ("晚制动回放", _late_html(), stills),
        ],
    )
    print(f"报告 {REPORT_PATH}", flush=True)


def _select(cases: list[dict]) -> list[dict]:
    chosen = []
    seen = set()
    for case in cases:
        if case["suspected_cause"] != "late_brake":
            continue
        template = case["manifest"]["template"]
        if template in seen:
            continue
        seen.add(template)
        chosen.append(case)
        if len(chosen) == 3:
            break
    if len(chosen) < 3:
        raise RuntimeError(f"晚制动线索只有 {len(chosen)} 条，不够抽查 3 条")
    for cause in ("late_discovery", "aes_infeasible"):
        for case in cases:
            if case["suspected_cause"] == cause:
                chosen.append(case)
                break
    for case in chosen:
        case["case_id"] = "{cause}_{scene}_{strategy}".format(
            cause=case["suspected_cause"],
            scene=case["scene_id"],
            strategy=case["strategy_name"],
        )
    return chosen


def _write_cases(cases: list[dict]) -> None:
    refuse_test_write(CASE_DIR)
    if CASE_DIR.exists():
        for path in CASE_DIR.glob("*.json"):
            path.unlink()
    CASE_DIR.mkdir(parents=True, exist_ok=True)
    for case in cases:
        path = CASE_DIR / f"{case['case_id']}.json"
        refuse_test_write(path)
        path.write_text(json.dumps(case, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _check_late_brake(cases: list[dict], rows: list[dict]) -> None:
    lines = ["# 晚制动回放核对", ""]
    by_id = {row["case_id"]: row for row in rows}
    for case in cases:
        if case["suspected_cause"] != "late_brake":
            continue
        row = by_id[case["case_id"]]
        evidence = case["evidence"]
        replay = row["replay"]
        same = (
            replay["first_risk_frame"] == evidence["first_risk_frame"]
            and replay["avoidable_critical_frame"] == evidence["avoidable_critical_frame"]
            and replay["brake_request_frame"] == evidence["brake_request_frame"]
        )
        reasonable = (
            replay["first_risk_frame"] is not None
            and replay["avoidable_critical_frame"] is not None
            and replay["brake_request_frame"] is not None
            and replay["first_risk_frame"] <= replay["avoidable_critical_frame"]
            and replay["brake_request_frame"] > replay["avoidable_critical_frame"]
        )
        lines.append(
            "- {case}：原日志风险帧 {risk}，可避免末帧 {critical}，请求帧 {request}。"
            "回放分别是 {r1}、{r2}、{r3}。帧号一致：{same}。仍符合晚制动线索：{ok}。".format(
                case=case["case_id"],
                risk=evidence["first_risk_frame"],
                critical=evidence["avoidable_critical_frame"],
                request=evidence["brake_request_frame"],
                r1=replay["first_risk_frame"],
                r2=replay["avoidable_critical_frame"],
                r3=replay["brake_request_frame"],
                same="是" if same else "否",
                ok="是" if reasonable else "否",
            )
        )
    path = ROOT / "runs" / "round10" / "late_brake_check.md"
    refuse_test_write(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _cause_html(cases: list[dict]) -> str:
    counts: dict[str, int] = {}
    for case in cases:
        counts[case["suspected_cause"]] = counts.get(case["suspected_cause"], 0) + 1
    rows = "".join(f"<tr><td>{CAUSE_TEXT.get(key, key)}</td><td>{value}</td></tr>" for key, value in counts.items())
    return (
        "<p>suspected_cause 是线索，不是结论。证据数字写在每条案例的 evidence 里。"
        "晚制动指风险在可避免末帧之前已经有效，制动请求却晚于这一帧。</p>"
        f"<table><tr><th>线索</th><th>条数</th></tr>{rows}</table>"
    )


def _neighbor_html(generated: list[dict]) -> str:
    return (
        f"<p>新场景 {len(generated)} 条，写在 configs/scenarios/round9/neighborhood/tuning，split 是 tuning。"
        "字段和原来的网格一样，第 9 轮的 load_split 可以直接读。出现时机只记在 sampling.json，没有塞进场景文件。</p>"
    )


def _regress_html(rows: list[dict]) -> str:
    body = "".join(
        "<tr><td>{case}</td><td>{cause}</td><td>{status}</td></tr>".format(
            case=row["case_id"],
            cause=CAUSE_TEXT.get(row["suspected_cause"], row["suspected_cause"]),
            status={"new_failure": "新增失败", "fixed": "已修复", "unchanged": "未变"}[row["status"]],
        )
        for row in rows
    )
    return "<p>和案例里保存的上一次碰撞结果比较。红色是新增失败，绿色是已修复，灰色是未变。</p>" + f"<table><tr><th>案例</th><th>线索</th><th>差异</th></tr>{body}</table>"


def _late_html() -> str:
    path = ROOT / "runs" / "round10" / "late_brake_check.md"
    text = path.read_text(encoding="utf-8") if path.is_file() else ""
    return "<pre>" + text.replace("<", "") + "</pre>"


if __name__ == "__main__":
    main()
