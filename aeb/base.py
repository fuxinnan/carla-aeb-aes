"""两种 AEB 共用的转移记录。原因写进 events.jsonl，不能空。"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Transition:
    """一次状态变化。previous 和 current 是 AEB 状态名，reason 是这一拍跳转的原因。"""

    previous: str
    current: str
    reason: str
