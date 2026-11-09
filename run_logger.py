"""一次运行一个目录。帧、目标和事件只追加，不重写已写入的内容。"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Mapping

from schemas import (
    ActualControl,
    EpisodeSummary,
    FrameRecord,
    Track,
    dump_line,
    dump_pretty,
    parse_line,
    to_plain,
)


MANIFEST_NAME = "manifest.json"
FRAMES_NAME = "frames.jsonl"
TARGETS_NAME = "targets.jsonl"
EVENTS_NAME = "events.jsonl"
SUMMARY_NAME = "summary.json"
RUN_FILES = (MANIFEST_NAME, FRAMES_NAME, TARGETS_NAME, EVENTS_NAME, SUMMARY_NAME)


class RunLogger:
    """创建运行目录并追加记录。每条 jsonl 写入后 flush 并 fsync。"""

    def __init__(self, run_dir: str | Path, manifest: Mapping) -> None:
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.manifest = to_plain(dict(manifest))
        (self.run_dir / MANIFEST_NAME).write_text(dump_pretty(self.manifest), encoding="utf-8")
        self._frames = self._open_append(FRAMES_NAME)
        self._targets = self._open_append(TARGETS_NAME)
        self._events = self._open_append(EVENTS_NAME)
        self._closed = False

    def _open_append(self, name: str):
        path = self.run_dir / name
        path.touch()
        return path.open("a", encoding="utf-8", newline="\n")

    def log_frame(self, frame: FrameRecord) -> None:
        """追加一帧。返回前这条记录已经落盘。"""
        if not isinstance(frame, FrameRecord):
            raise TypeError("log_frame 需要 FrameRecord")
        self._append(self._frames, frame)

    def log_target(self, frame: int, sim_time_s: float, track: Track) -> None:
        """追加一条目标。一帧可以有多条。"""
        if not isinstance(track, Track):
            raise TypeError("log_target 需要 Track")
        self._append(
            self._targets,
            {"frame": frame, "sim_time_s": float(sim_time_s), "track": track},
        )

    def log_event(self, event_type: str, sim_time_s: float, detail: Mapping | None = None) -> None:
        """追加一条事件。detail 是可 JSON 化的普通对象。"""
        payload = {
            "event_type": str(event_type),
            "sim_time_s": float(sim_time_s),
            "detail": {} if detail is None else to_plain(dict(detail)),
        }
        self._append(self._events, payload)

    def update_manifest(self, extra: Mapping) -> None:
        """补写清单里要等跑完才知道的字段。只重写 manifest.json。"""
        self._check_open()
        self.manifest.update(to_plain(dict(extra)))
        (self.run_dir / MANIFEST_NAME).write_text(dump_pretty(self.manifest), encoding="utf-8")

    def write_summary(self, summary: EpisodeSummary) -> None:
        """写入 summary.json。只覆盖汇总文件，不动已经追加的帧。"""
        if not isinstance(summary, EpisodeSummary):
            raise TypeError("write_summary 需要 EpisodeSummary")
        self._check_open()
        (self.run_dir / SUMMARY_NAME).write_text(dump_pretty(summary), encoding="utf-8")

    def read_frames(self) -> list[FrameRecord]:
        """按文件顺序读回全部帧。"""
        records = []
        raw = (self.run_dir / FRAMES_NAME).read_bytes()
        for line in raw.splitlines(keepends=True):
            records.append(parse_line(FrameRecord, line.decode("utf-8")))
        return records

    def close(self) -> None:
        if self._closed:
            return
        for handle in (self._frames, self._targets, self._events):
            handle.flush()
            handle.close()
        self._closed = True

    def __enter__(self) -> "RunLogger":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def _append(self, handle, payload) -> None:
        self._check_open()
        handle.write(dump_line(payload))
        handle.flush()
        os.fsync(handle.fileno())

    def _check_open(self) -> None:
        if self._closed:
            raise RuntimeError("运行日志已经关闭")
