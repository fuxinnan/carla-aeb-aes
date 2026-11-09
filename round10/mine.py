"""从一次运行的日志里挑难例，并给一个待核对的原因线索。"""

from __future__ import annotations

import json
from pathlib import Path

import yaml

from metrics.episode import evaluate_run
from round10.guard import ROOT


CAUSE_TEXT = {
    "late_discovery": "晚发现",
    "late_brake": "晚制动",
    "slow_response": "响应慢",
    "false_trigger": "误触发",
    "aes_infeasible": "AES 不可行",
    "tracking_error": "跟踪误差大",
    "data_gap": "数据缺帧",
    "unspecified": "无线索",
}


def load_thresholds(path: str | Path | None = None) -> dict:
    source = Path(path) if path is not None else ROOT / "configs" / "round10" / "mining.yaml"
    loaded = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError(f"{source} 必须是映射")
    return loaded


def iter_run_dirs(roots: list[str | Path]) -> list[Path]:
    found = []
    for root in roots:
        base = ROOT / root if not Path(root).is_absolute() else Path(root)
        if not base.is_dir():
            continue
        for summary in sorted(base.glob("**/summary.json")):
            run_dir = summary.parent
            if (run_dir / "frames.jsonl").is_file() and (run_dir / "events.jsonl").is_file() and (run_dir / "manifest.json").is_file():
                found.append(run_dir)
    return found


def inspect_run(run_dir: str | Path, thresholds: dict) -> dict | None:
    """读日志。不够格就返回 None。suspected_cause 只是线索。"""
    root = Path(run_dir)
    frames = _read_jsonl(root / "frames.jsonl")
    events = _read_jsonl(root / "events.jsonl")
    summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))
    metrics = evaluate_run(root)
    timing = _timing(frames)
    evidence = {
        "collided": bool(summary["collided"]),
        "min_distance_m": summary.get("min_distance_m"),
        "state_switch_count": metrics["state_switch_count"],
        "false_brake": metrics["false_brake"],
        "request_to_decel_frames": metrics["request_to_decel_frames"],
        "max_abs_path_error_m": timing["max_abs_path_error_m"],
        "aes_infeasible_frames": timing["aes_infeasible_frames"],
        "gap_frames": timing["gap_frames"],
        "first_risk_frame": timing["first_risk_frame"],
        "avoidable_critical_frame": timing["critical_frame"],
        "brake_request_frame": timing["request_frame"],
        "risk_before_or_at_critical": timing["risk_known_in_time"],
    }
    rules = _rules(evidence, thresholds)
    if not rules:
        return None
    cause = _cause(evidence, thresholds)
    spec = _spec(events)
    manifest = _manifest(root, summary, spec)
    return {
        "run_dir": str(root),
        "scene_id": str(summary["scene_id"]),
        "strategy_name": str(summary["strategy_name"]),
        "ego_speed_mps": spec["ego_speed_mps"],
        "distance_m": spec["distance_m"],
        "appearance_s": None if timing["first_risk_frame"] is None else timing["first_risk_frame"] * 0.05,
        "hard_rules": rules,
        "suspected_cause": cause,
        "evidence": evidence,
        "baseline": {
            "collided": bool(summary["collided"]),
            "min_distance_m": summary.get("min_distance_m"),
            "outcome": summary.get("outcome"),
        },
        "manifest": manifest,
    }


def _rules(evidence: dict, thresholds: dict) -> list[str]:
    rules = []
    if evidence["collided"]:
        rules.append("collision")
    gap = evidence["min_distance_m"]
    if not evidence["collided"] and gap is not None and gap < float(thresholds["near_miss_m"]):
        rules.append("near_miss")
    if evidence["state_switch_count"] >= int(thresholds["state_switch_count"]):
        rules.append("state_chatter")
    if evidence["aes_infeasible_frames"]:
        rules.append("aes_infeasible")
    error = evidence["max_abs_path_error_m"]
    if error is not None and error > float(thresholds["path_error_m"]):
        rules.append("tracking_error")
    if evidence["gap_frames"]:
        rules.append("data_gap")
    return rules


def _cause(evidence: dict, thresholds: dict) -> str:
    """按线索优先级给一个 suspected_cause。数字在 evidence 里，不写成定论。"""
    if evidence["false_brake"] is True:
        return "false_trigger"
    risk = evidence["first_risk_frame"]
    critical = evidence["avoidable_critical_frame"]
    request = evidence["brake_request_frame"]
    if (
        evidence["risk_before_or_at_critical"]
        and critical is not None
        and request is not None
        and request > critical
    ):
        return "late_brake"
    if evidence["aes_infeasible_frames"]:
        return "aes_infeasible"
    if risk is not None and critical is None and evidence["collided"]:
        return "late_discovery"
    if risk is not None and critical is not None and risk > critical:
        return "late_discovery"
    delay = evidence["request_to_decel_frames"]
    if delay is not None and delay >= int(thresholds["slow_response_frames"]):
        return "slow_response"
    if evidence["aes_infeasible_frames"]:
        return "aes_infeasible"
    error = evidence["max_abs_path_error_m"]
    if error is not None and error > float(thresholds["path_error_m"]):
        return "tracking_error"
    if evidence["gap_frames"]:
        return "data_gap"
    return "unspecified"


def _timing(frames: list[dict]) -> dict:
    first_risk = None
    request = None
    critical = None
    last_true = None
    seen = False
    max_error = None
    infeasible = []
    gaps = []
    previous = None
    for frame in frames:
        number = int(frame["frame"])
        if previous is not None and number != previous + 1:
            gaps.append(number)
        previous = number
        threat = frame.get("threat") or {}
        if first_risk is None and threat.get("ttc_s") is not None:
            first_risk = number
            seen = True
        if seen and threat.get("brake_avoidable") is True:
            last_true = number
        if seen and threat.get("brake_avoidable") is False and last_true is not None and critical is None:
            critical = last_true
        decel = (frame.get("control_request") or {}).get("target_decel_mps2")
        if request is None and decel is not None and float(decel) >= 0.5:
            request = number
        error = frame.get("path_error_m")
        if error is not None:
            max_error = abs(float(error)) if max_error is None else max(max_error, abs(float(error)))
        reason = frame.get("arbitration_reason") or ""
        if "aes_infeasible" in reason:
            infeasible.append(number)
    return {
        "first_risk_frame": first_risk,
        "request_frame": request,
        "critical_frame": critical,
        "risk_known_in_time": first_risk is not None and critical is not None and first_risk <= critical,
        "max_abs_path_error_m": max_error,
        "aes_infeasible_frames": infeasible,
        "gap_frames": gaps,
    }


def _spec(events: list[dict]) -> dict:
    for event in events:
        if event.get("event_type") == "scenario_spec":
            detail = event.get("detail") or {}
            return {
                "template": detail.get("template"),
                "instance": detail.get("instance"),
                "seed": detail.get("seed"),
                "split": detail.get("split"),
                "ego_speed_mps": float((detail.get("ego") or {}).get("speed_mps")),
                "distance_m": float((detail.get("target") or {}).get("gap_m")),
                "hard_negative": bool((detail.get("end") or {}).get("hard_negative")),
            }
    raise ValueError("日志里没有 scenario_spec")


def _manifest(run_dir: Path, summary: dict, spec: dict) -> dict:
    template = spec["template"]
    return {
        "scene_id": summary["scene_id"],
        "strategy_name": summary["strategy_name"],
        "split": spec["split"],
        "seed": spec["seed"],
        "template": template,
        "instance": spec["instance"],
        "ego_speed_mps": spec["ego_speed_mps"],
        "distance_m": spec["distance_m"],
        "hard_negative": spec["hard_negative"],
        "occlusion": template == "occlusion_reveal",
        "aeb_config": _config_for(run_dir, summary["strategy_name"]),
    }


def _config_for(run_dir: Path, strategy: str) -> str:
    parts = run_dir.parts
    if "search" in parts:
        name = parts[parts.index("search") + 1]
        return f"configs/aeb/round9/search/{name}.yaml"
    if "ablation" in parts:
        name = parts[parts.index("ablation") + 1]
        return f"configs/aeb/round9/ablate_{name}.yaml"
    if strategy == "fixed_ttc":
        return "configs/aeb/round9/chosen_fixed.yaml"
    if strategy in {"state_machine", "aeb_aes"}:
        return "configs/aeb/round9/chosen_state.yaml"
    return "configs/aeb/round5.yaml"


def _read_jsonl(path: Path) -> list[dict]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows
