"""运行目录的公共出图。只读 manifest、帧、目标、事件和汇总，不连接仿真器。"""

from __future__ import annotations

import hashlib
import json
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

# 直接运行 viz/common.py 时，sys.path 只有 viz/。先补上项目根，才能导入同级模块。
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if not any(Path(entry).resolve() == _PROJECT_ROOT for entry in sys.path if entry):
    sys.path.insert(0, str(_PROJECT_ROOT))

import numpy as np
import pandas as pd

from run_logger import EVENTS_NAME, FRAMES_NAME, MANIFEST_NAME, SUMMARY_NAME, TARGETS_NAME
from schemas import AEB_STATES, FrameRecord, parse_line


# IDLE 灰，WARN 黄，PARTIAL 橙，FULL 红，EXIT 蓝。后面的图都用这一份。
AEB_STATE_COLORS = {
    "IDLE": "#9E9E9E",
    "WARN": "#F9A825",
    "PARTIAL": "#FB8C00",
    "FULL": "#E53935",
    "EXIT": "#1E88E5",
}
UNKNOWN_STATE_COLOR = "#212121"

# 已知策略的固定颜色。未列入的名字按名称哈希进色板，同名每次一样。
STRATEGY_COLORS = {
    "aeb": "#E53935",
    "aes": "#43A047",
    "aeb_aes": "#8E24AA",
    "baseline": "#546E7A",
    "no_intervention": "#546E7A",
    "fixed_ttc": "#C62828",
    "state_machine": "#1565C0",
    "driver": "#6D4C41",
}
_STRATEGY_PALETTE = (
    "#00897B",
    "#3949AB",
    "#6D4C41",
    "#C0CA33",
    "#8D6E63",
    "#5E35B1",
    "#039BE5",
    "#F4511E",
)

_FONT_READY = False
_UNSAFE_FILENAME = re.compile(r'[<>:"/\\|?*\s]+')


@dataclass(frozen=True)
class RunData:
    """五个文件的读回结果。三张表给作图，清单和汇总原样保留。"""

    frames: pd.DataFrame
    targets: pd.DataFrame
    events: pd.DataFrame
    manifest: dict
    summary: dict


def aeb_state_color(state: str) -> str:
    """AEB 状态色。未知状态用深灰，避免画图中断。"""
    return AEB_STATE_COLORS.get(state, UNKNOWN_STATE_COLOR)


def strategy_color(name: str) -> str:
    """策略色。已知策略用固定色，其余按名称稳定取色。"""
    key = name.strip().lower()
    if key in STRATEGY_COLORS:
        return STRATEGY_COLORS[key]
    digest = hashlib.md5(key.encode("utf-8")).hexdigest()
    return _STRATEGY_PALETTE[int(digest[:8], 16) % len(_STRATEGY_PALETTE)]


def load_run(run_dir: str | Path) -> RunData:
    """读回五个文件，返回帧表、目标表、事件表，以及清单和汇总。"""
    root = Path(run_dir)
    missing = [name for name in (MANIFEST_NAME, FRAMES_NAME, TARGETS_NAME, EVENTS_NAME, SUMMARY_NAME) if not (root / name).is_file()]
    if missing:
        raise FileNotFoundError(f"{root} 缺少文件: {missing}")

    manifest = json.loads((root / MANIFEST_NAME).read_text(encoding="utf-8"))
    summary = json.loads((root / SUMMARY_NAME).read_text(encoding="utf-8"))
    frames = _frames_table(root / FRAMES_NAME)
    targets = _object_table(root / TARGETS_NAME)
    events = _object_table(root / EVENTS_NAME)
    return RunData(frames=frames, targets=targets, events=events, manifest=manifest, summary=summary)


def save_fig(fig, run_dir: str | Path, name: str) -> Path:
    """把图存到运行目录的 figures/。文件名带场景编号。"""
    root = Path(run_dir)
    manifest = json.loads((root / MANIFEST_NAME).read_text(encoding="utf-8"))
    scene_id = manifest["scene_id"]
    if not isinstance(scene_id, str) or not scene_id.strip():
        raise ValueError("manifest.json 的 scene_id 必须是非空字符串")
    figures = root / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    filename = f"{_safe_token(scene_id)}_{_safe_token(name)}.png"
    path = figures / filename
    fig.savefig(path, dpi=120, bbox_inches="tight")
    return path


def plot_run_overview(run_dir: str | Path) -> Path:
    """画速度、减速度、TTC 和 AEB 状态色带，四幅图共用时间轴。"""
    _prepare_matplotlib()
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    run = load_run(run_dir)
    frame_table = run.frames
    if frame_table.empty:
        raise ValueError(f"{run_dir} 没有帧，无法画总览")

    time_s = frame_table["sim_time_s"].to_numpy(dtype=float)
    speed = frame_table["ego_speed_mps"].to_numpy(dtype=float)
    deceleration = -frame_table["ego_a_long_mps2"].to_numpy(dtype=float)
    ttc = frame_table["threat_ttc_s"].to_numpy(dtype=float)
    states = frame_table["aeb_state"].astype(str).tolist()
    strategy_name = str(frame_table["strategy_name"].iloc[0])
    scene_id = str(run.manifest["scene_id"])

    figure, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 8))
    speed_axis, decel_axis, ttc_axis, state_axis = axes
    line_color = strategy_color(strategy_name)

    speed_axis.plot(time_s, speed, color=line_color, linewidth=1.8)
    speed_axis.set_ylabel("速度 (m/s)")
    speed_axis.grid(True, alpha=0.3)

    decel_axis.plot(time_s, deceleration, color=line_color, linewidth=1.8)
    decel_axis.set_ylabel("减速度 (m/s²)")
    decel_axis.grid(True, alpha=0.3)

    ttc_axis.plot(time_s, ttc, color=line_color, linewidth=1.8)
    ttc_axis.set_ylabel("TTC (s)")
    ttc_axis.grid(True, alpha=0.3)

    widths = _bar_widths(time_s)
    speed_axis.set_xlim(float(time_s[0]), float(time_s[-1] + widths[-1]))
    colors = [aeb_state_color(state) for state in states]
    state_axis.bar(time_s, np.ones(len(time_s)), width=widths, color=colors, align="edge", linewidth=0)
    state_axis.set_ylabel("AEB")
    state_axis.set_ylim(0.0, 1.0)
    state_axis.set_yticks([])
    state_axis.set_xlabel("仿真时间 (s)")
    figure.legend(
        handles=[
            Patch(facecolor=AEB_STATE_COLORS[state], edgecolor="none", label=state)
            for state in AEB_STATES
        ],
        loc="lower center",
        ncol=5,
        frameon=False,
        bbox_to_anchor=(0.5, 0.0),
    )
    figure.suptitle(f"{scene_id} / {strategy_name}")
    figure.tight_layout(rect=(0.0, 0.05, 1.0, 0.96))
    path = save_fig(figure, run_dir, "overview")
    plt.close(figure)
    return path


def _frames_table(path: Path) -> pd.DataFrame:
    records = []
    raw = path.read_bytes()
    if not raw:
        return pd.DataFrame()
    for line in raw.splitlines(keepends=True):
        text = line.decode("utf-8")
        parse_line(FrameRecord, text)
        records.append(json.loads(text))
    table = pd.json_normalize(records, sep="_")
    if "frame" in table.columns:
        table = table.sort_values("frame", kind="stable").reset_index(drop=True)
    return table


def _object_table(path: Path) -> pd.DataFrame:
    raw = path.read_text(encoding="utf-8")
    if not raw.strip():
        return pd.DataFrame()
    rows = [json.loads(line) for line in raw.splitlines() if line.strip()]
    return pd.json_normalize(rows, sep="_")


def _bar_widths(time_s: np.ndarray) -> np.ndarray:
    if len(time_s) == 1:
        return np.array([0.05])
    steps = np.diff(time_s)
    last = steps[-1] if len(steps) else 0.05
    return np.append(steps, last)


def _safe_token(value: str) -> str:
    cleaned = _UNSAFE_FILENAME.sub("_", value.strip())
    return cleaned or "unnamed"


def _prepare_matplotlib() -> None:
    global _FONT_READY
    import matplotlib
    from matplotlib import font_manager

    matplotlib.rcParams["axes.unicode_minus"] = False
    if _FONT_READY:
        return
    font_file = Path(r"C:\Windows\Fonts\msyh.ttc")
    if font_file.is_file():
        font_manager.fontManager.addfont(str(font_file))
        matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
        matplotlib.rcParams["font.family"] = "sans-serif"
    _FONT_READY = True
