"""把已有对照表、图和难例关键帧收成一个不联网的看板。"""

from __future__ import annotations

import base64
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "dashboard" / "dashboard.html"


def main() -> None:
    html = _page()
    OUTPUT.write_text(html, encoding="utf-8")
    print(f"写出 {OUTPUT}", flush=True)


def _page() -> str:
    overview = _overview()
    chapters = [
        ("第 6 轮 调参", ROOT / "runs" / "round6" / "tuning"),
        ("第 6 轮 测试", ROOT / "runs" / "round6" / "test"),
        ("第 9 轮", None),
    ]
    parts = [
        "<!DOCTYPE html><html lang='zh-CN'><head><meta charset='utf-8'><title>AEB/AES 看板</title>",
        "<style>body{font-family:'Microsoft YaHei',sans-serif;margin:32px;color:#212121;background:#fafafa;}",
        "table{border-collapse:collapse;margin:12px 0 20px;background:#fff;}td,th{border:1px solid #B0BEC5;padding:6px 10px;text-align:left;}",
        "img{max-width:100%;margin:8px 0 16px;}details{background:#fff;border:1px solid #CFD8DC;border-radius:8px;margin:12px 0;padding:8px 16px;}",
        "summary{cursor:pointer;font-weight:600;padding:6px 0;} .cards{display:flex;flex-wrap:wrap;gap:12px;}",
        ".card{background:#fff;border:1px solid #CFD8DC;border-radius:8px;padding:12px 16px;min-width:140px;}",
        ".card b{display:block;font-size:22px;}</style></head><body>",
        "<h1>AEB/AES 汇总看板</h1>",
        "<p>图和表都嵌在这一个文件里，打开时不访问网络。</p>",
        "<h2>项目概览</h2>",
        overview,
        "<h2>对照和图</h2>",
    ]
    parts.append(_chapter("第 6 轮 调参", _markdown_tables(ROOT / "runs" / "round6" / "tuning" / "comparison.md"), _images(ROOT / "runs" / "round6" / "tuning", ("overview.png", "delays.png", "outcome_grid.png"))))
    parts.append(_chapter("第 6 轮 测试", _markdown_tables(ROOT / "runs" / "round6" / "test" / "comparison.md"), _images(ROOT / "runs" / "round6" / "test", ("overview.png", "delays.png", "outcome_grid.png"))))
    parts.append(_chapter("第 9 轮", _markdown_tables(ROOT / "runs" / "round9" / "comparison.md"), _round9_images()))
    parts.append("<h2>难例</h2>")
    parts.append(_hard_cases())
    parts.append("</body></html>")
    del chapters
    return "\n".join(parts)


def _overview() -> str:
    templates = sorted((ROOT / "scenarios" / "templates").glob("*.yaml"))
    instances = sorted((ROOT / "configs" / "scenarios").rglob("*.yaml"))
    later = list((ROOT / "runs" / "round6").rglob("manifest.json")) + list((ROOT / "runs" / "round9").rglob("manifest.json"))
    strategies = set()
    sample = None
    for path in later:
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("strategy_name"):
            strategies.add(str(data["strategy_name"]))
        if sample is None and data.get("map") == "Town10HD_Opt":
            sample = data
    cards = [
        ("场景模板", str(len(templates))),
        ("场景实例", str(len(instances))),
        ("策略", str(len(strategies))),
        ("第 6 轮重复", _repeat_span(ROOT / "runs" / "round6")),
        ("第 9 轮测试", _repeat_span(ROOT / "runs" / "round9" / "logs" / "test")),
        ("第 9 轮稳定性", _repeat_span(ROOT / "runs" / "round9" / "logs" / "stability")),
    ]
    html = "<div class='cards'>" + "".join(f"<div class='card'><b>{value}</b>{label}</div>" for label, value in cards) + "</div>"
    if sample is None:
        return html + "<p>没有带地图字段的 manifest。</p>"
    rows = [
        ("schema_version", sample.get("schema_version")),
        ("scene_id", sample.get("scene_id")),
        ("strategy_name", sample.get("strategy_name")),
        ("strategy_version", sample.get("strategy_version")),
        ("map", sample.get("map")),
        ("carla_version", sample.get("carla_version")),
        ("synchronous_mode", sample.get("synchronous_mode")),
        ("fixed_delta_seconds", sample.get("fixed_delta_seconds")),
        ("seeds", json.dumps(sample.get("seeds"), ensure_ascii=False)),
    ]
    body = "".join(f"<tr><td>{name}</td><td>{value}</td></tr>" for name, value in rows)
    return html + "<p>下面是一份带 CARLA 地图的 manifest 摘要。第 9 轮的运动学清单只有场景、策略、划分和种子。</p><table><tr><th>字段</th><th>值</th></tr>" + body + "</table>"


def _repeat_span(directory: Path) -> str:
    names = sorted({path.parent.name for path in directory.rglob("manifest.json") if path.parent.name.startswith("rep_")})
    return "、".join(names) if names else "—"


def _chapter(title: str, tables: str, images: list[str]) -> str:
    return f"<details open><summary>{title}</summary>{tables}{''.join(images)}</details>"


def _round9_images() -> list[str]:
    names = (
        "grid_tuning_no_intervention.png",
        "grid_tuning_fixed_ttc.png",
        "grid_tuning_state_machine.png",
        "grid_tuning_aeb_aes.png",
        "grid_test_no_intervention.png",
        "grid_test_fixed_ttc.png",
        "grid_test_state_machine.png",
        "grid_test_aeb_aes.png",
        "ablation.png",
        "stability.png",
        "occlusion.png",
    )
    return _images(ROOT / "figures" / "round9", names)


def _images(directory: Path, names: tuple[str, ...]) -> list[str]:
    tags = []
    for name in names:
        path = directory / name
        if not path.is_file():
            continue
        encoded = base64.b64encode(path.read_bytes()).decode("ascii")
        tags.append(f"<figure><figcaption>{name}</figcaption><img alt='{name}' src='data:image/png;base64,{encoded}'></figure>")
    return tags


def _markdown_tables(path: Path) -> str:
    if not path.is_file():
        return "<p>没有对照表。</p>"
    tables = []
    rows: list[list[str]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("|"):
            cells = [cell.strip() for cell in line.strip("|").split("|")]
            if all(set(cell) <= set("-: ") for cell in cells):
                continue
            rows.append(cells)
            continue
        if rows:
            tables.append(_table(rows))
            rows = []
    if rows:
        tables.append(_table(rows))
    return "\n".join(tables)


def _table(rows: list[list[str]]) -> str:
    head = "".join(f"<th>{_cell(cell)}</th>" for cell in rows[0])
    body = []
    for row in rows[1:]:
        body.append("<tr>" + "".join(f"<td>{_cell(cell)}</td>" for cell in row) + "</tr>")
    return "<table><tr>" + head + "</tr>" + "".join(body) + "</table>"


def _cell(value: str) -> str:
    return value.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def _hard_cases() -> str:
    case_dir = ROOT / "regression" / "cases"
    blocks = []
    for path in sorted(case_dir.glob("*.json")):
        case = json.loads(path.read_text(encoding="utf-8"))
        evidence = case.get("evidence") or {}
        summary = (
            f"{case['case_id']} · 线索 {case.get('suspected_cause')} · "
            f"风险帧 {evidence.get('first_risk_frame')} · 可避免末帧 {evidence.get('avoidable_critical_frame')} · "
            f"请求帧 {evidence.get('brake_request_frame')}"
        )
        frames = _case_frames(case["case_id"])
        blocks.append(f"<details><summary>{_cell(summary)}</summary>{''.join(frames) or '<p>这条没有导出关键帧。</p>'}</details>")
    if not blocks:
        return "<p>regression/cases 里还没有难例。</p>"
    return "\n".join(blocks)


def _case_frames(case_id: str) -> list[str]:
    directory = ROOT / "figures" / "round10" / "replay" / case_id
    tags = []
    for name in ("risk", "brake", "closest"):
        path = directory / f"{case_id}_{name}.png"
        if not path.is_file():
            continue
        encoded = base64.b64encode(path.read_bytes()).decode("ascii")
        label = {"risk": "风险出现", "brake": "制动开始", "closest": "最近点"}[name]
        tags.append(f"<figure><figcaption>{label}</figcaption><img alt='{label}' src='data:image/png;base64,{encoded}'></figure>")
    return tags


if __name__ == "__main__":
    main()
