"""用清单重跑回归集，和上一次的碰撞结果比。"""

from __future__ import annotations

import json
from pathlib import Path

from round10.guard import ROOT, refuse_test_write
from round10.mine import _timing
from round10.replay import replay


CASE_DIR = ROOT / "regression" / "cases"
LAST_PATH = ROOT / "regression" / "last_result.json"
DIFF_PATH = ROOT / "regression" / "diff.md"


def run_regression(case_dir: str | Path | None = None, last_path: str | Path | None = None, stage=None) -> list[dict]:
    """一条命令的入口。没有上次结果时，用案例里保存的原始日志结果。缺省会自己打开 CARLA。"""
    if stage is None:
        from sim.carla_stage import CarlaStage

        with CarlaStage() as owned:
            return _run_regression(case_dir, last_path, owned)
    return _run_regression(case_dir, last_path, stage)


def _run_regression(case_dir: str | Path | None, last_path: str | Path | None, stage) -> list[dict]:
    directory = Path(case_dir) if case_dir is not None else CASE_DIR
    refuse_test_write(directory)
    previous = _previous(Path(last_path) if last_path is not None else LAST_PATH, directory)
    rows = []
    for path in sorted(directory.glob("*.json")):
        case = json.loads(path.read_text(encoding="utf-8"))
        result = replay(case["manifest"], stage)
        timing = _timing(_frames_from_result(result))
        current = {
            "collided": bool(result.collided),
            "min_distance_m": result.summary.min_distance_m,
            "outcome": result.summary.outcome,
            "first_risk_frame": timing["first_risk_frame"],
            "avoidable_critical_frame": timing["critical_frame"],
            "brake_request_frame": timing["request_frame"],
        }
        before = previous.get(case["case_id"], case["baseline"])
        rows.append(
            {
                "case_id": case["case_id"],
                "scene_id": case["manifest"]["scene_id"],
                "strategy_name": case["manifest"]["strategy_name"],
                "suspected_cause": case["suspected_cause"],
                "previous_collided": bool(before["collided"]),
                "current_collided": current["collided"],
                "status": _status(bool(before["collided"]), current["collided"]),
                "previous_min_distance_m": before.get("min_distance_m"),
                "current_min_distance_m": current["min_distance_m"],
                "replay": current,
            }
        )
    payload = {"rows": rows}
    destination = LAST_PATH if last_path is None else Path(last_path)
    refuse_test_write(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    text = _markdown(rows)
    diff_path = destination.parent / "diff.md"
    refuse_test_write(diff_path)
    diff_path.write_text(text, encoding="utf-8")
    return rows


def _previous(last_path: Path, case_dir: Path) -> dict:
    if last_path.is_file():
        loaded = json.loads(last_path.read_text(encoding="utf-8"))
        return {
            row["case_id"]: {"collided": row["current_collided"], "min_distance_m": row["current_min_distance_m"]}
            for row in loaded.get("rows", [])
        }
    found = {}
    for path in case_dir.glob("*.json"):
        case = json.loads(path.read_text(encoding="utf-8"))
        found[case["case_id"]] = case["baseline"]
    return found


def _status(before: bool, after: bool) -> str:
    if not before and after:
        return "new_failure"
    if before and not after:
        return "fixed"
    return "unchanged"


def _markdown(rows: list[dict]) -> str:
    lines = [
        "| 案例 | 线索 | 上次碰撞 | 这次碰撞 | 差异 |",
        "| --- | --- | --- | --- | --- |",
    ]
    text = {"new_failure": "新增失败", "fixed": "已修复", "unchanged": "未变", "late_brake": "晚制动", "late_discovery": "晚发现", "aes_infeasible": "AES 不可行", "false_trigger": "误触发", "slow_response": "响应慢", "tracking_error": "跟踪误差大", "data_gap": "数据缺帧", "unspecified": "无线索"}
    for row in rows:
        lines.append(
            "| {case} | {cause} | {before} | {after} | {status} |".format(
                case=row["case_id"],
                cause=text.get(row["suspected_cause"], row["suspected_cause"]),
                before="1" if row["previous_collided"] else "0",
                after="1" if row["current_collided"] else "0",
                status=text[row["status"]],
            )
        )
    return "\n".join(lines) + "\n"


def _frames_from_result(result) -> list[dict]:
    frames = []
    for record in result.records:
        frames.append(
            {
                "frame": record.frame,
                "threat": {
                    "ttc_s": record.threat.ttc_s,
                    "brake_avoidable": record.threat.brake_avoidable,
                },
                "control_request": {"target_decel_mps2": record.control_request.target_decel_mps2},
                "path_error_m": record.path_error_m,
                "arbitration_reason": record.arbitration_reason,
            }
        )
    return frames


if __name__ == "__main__":
    result = run_regression()
    print(f"回归 {len(result)} 条，差异表 {DIFF_PATH}")
