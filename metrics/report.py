"""单文件 HTML。图用 base64 嵌进去，打开时不依赖旁边的文件。"""

from __future__ import annotations

import base64
import html
from pathlib import Path

from metrics.aggregate import comparison_markdown, investigation_note, write_comparison_markdown, write_episode_csv
from metrics.episode import evaluate_run
from metrics.plots import plot_delay_boxes, plot_outcome_grid, plot_overview


def collect_runs(root: str | Path) -> list[dict]:
    """找出目录下所有带 frames.jsonl 的运行，并计算指标。"""
    base = Path(root)
    runs = sorted(path.parent for path in base.rglob("frames.jsonl"))
    return [evaluate_run(path) for path in runs]


def build_report(rows: list[dict], output_dir: str | Path, title: str, split_note: str) -> Path:
    """写 CSV、Markdown、三张图，以及把它们嵌进 report.html。"""
    folder = Path(output_dir)
    folder.mkdir(parents=True, exist_ok=True)
    write_episode_csv(rows, folder / "episodes.csv")
    markdown = comparison_markdown(rows)
    write_comparison_markdown(rows, folder / "comparison.md")
    overview = plot_overview(rows, folder / "overview.png")
    delays = plot_delay_boxes(rows, folder / "delays.png")
    grid = plot_outcome_grid(rows, folder / "outcome_grid.png")
    note = investigation_note(rows)
    document = _document(title, split_note, markdown, note, [overview, delays, grid])
    target = folder / "report.html"
    target.write_text(document, encoding="utf-8")
    return target


def _document(title: str, split_note: str, markdown: str, note: str, images: list[Path]) -> str:
    figures = "\n".join(
        f'<img alt="{html.escape(path.stem)}" src="data:image/png;base64,{_encode(path)}"/>'
        for path in images
    )
    body = _markdown_tables(markdown)
    return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<title>{html.escape(title)}</title>
<style>
body {{ font-family: "Microsoft YaHei", sans-serif; margin: 28px; color: #212121; }}
table {{ border-collapse: collapse; margin: 12px 0 24px; }}
th, td {{ border: 1px solid #bdbdbd; padding: 4px 8px; text-align: left; }}
img {{ display: block; max-width: 100%; margin: 12px 0 24px; }}
pre {{ white-space: pre-wrap; }}
</style>
</head>
<body>
<h1>{html.escape(title)}</h1>
<p>{html.escape(split_note)}</p>
{body}
<h2>日志核对</h2>
<pre>{html.escape(note)}</pre>
{figures}
</body>
</html>
"""


def _markdown_tables(markdown: str) -> str:
    blocks = []
    rows: list[str] = []
    for line in markdown.splitlines():
        if line.startswith("|"):
            rows.append(line)
            continue
        if rows:
            blocks.append(_table(rows))
            rows = []
    if rows:
        blocks.append(_table(rows))
    return "\n".join(blocks)


def _table(lines: list[str]) -> str:
    parsed = [_split_row(line) for line in lines if not _is_rule(line)]
    if not parsed:
        return ""
    head = "".join(f"<th>{html.escape(cell)}</th>" for cell in parsed[0])
    body = []
    for row in parsed[1:]:
        body.append("<tr>" + "".join(f"<td>{html.escape(cell)}</td>" for cell in row) + "</tr>")
    return "<table><thead><tr>" + head + "</tr></thead><tbody>" + "".join(body) + "</tbody></table>"


def _split_row(line: str) -> list[str]:
    return [cell.strip() for cell in line.strip().strip("|").split("|")]


def _is_rule(line: str) -> bool:
    cells = _split_row(line)
    return bool(cells) and all(set(cell) <= set("-:") and cell for cell in cells)


def _encode(path: Path) -> str:
    return base64.b64encode(path.read_bytes()).decode("ascii")
