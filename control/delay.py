"""请求减速度到实测减速度的延迟。实测来自车速差，不看来自执行器的命令。"""

from __future__ import annotations

ONSET_DECEL_MPS2 = 0.5


def brake_onset(requested, measured, onset_mps2: float = ONSET_DECEL_MPS2) -> tuple[int, int] | None:
    """返回 (请求帧, 实测帧)。实测帧必须晚于请求帧。没有这样的一对时返回 None。"""
    request_frame = _first_at_least(requested, onset_mps2, 0)
    if request_frame is None:
        return None
    measured_frame = _first_at_least(measured, onset_mps2, request_frame + 1)
    if measured_frame is None:
        return None
    return request_frame, measured_frame


def brake_delay_frames(requested, measured, onset_mps2: float = ONSET_DECEL_MPS2) -> int | None:
    """延迟帧数。没有制动，或日志在实测出现前结束，都是 None。"""
    span = brake_onset(requested, measured, onset_mps2)
    if span is None:
        return None
    return span[1] - span[0]


def _first_at_least(values, onset_mps2: float, start: int) -> int | None:
    for index in range(start, len(values)):
        value = values[index]
        if value is None:
            continue
        if value >= onset_mps2:
            return index
    return None
