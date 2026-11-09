"""比较只制动和制动加避让。每帧都给出一个原因。"""

from __future__ import annotations

REASON_BRAKE_OK = "aeb_only_brake_sufficient"
REASON_NO_CONFLICT = "aeb_only_no_conflict"
REASON_AES = "aes_selected"
REASON_TRACK = "aes_tracking"
REASON_INFEASIBLE = "aes_infeasible"
REASON_DONE = "aes_completed"
REASON_FRICTION = "friction_limited"

REASON_TEXT = {
    REASON_BRAKE_OK: "制动来得及，只用 AEB",
    REASON_NO_CONFLICT: "没有需要制动的冲突",
    REASON_AES: "制动来不及且避让可行，开始转向",
    REASON_TRACK: "沿已选定的避让轨迹跟踪",
    REASON_INFEASIBLE: "制动来不及，避让轨迹也不可行",
    REASON_DONE: "避让轨迹走完",
    REASON_FRICTION: "附着圆缩小了制动和转向",
}


def arbitration_reason(brake_avoidable: bool | None, aes_feasible: bool, tracking: bool, finished: bool) -> str:
    """默认只用 AEB。只有制动距离不够而且避让轨迹可行时才转向。"""
    if finished:
        return REASON_DONE
    if tracking:
        return REASON_TRACK
    if brake_avoidable is False and aes_feasible:
        return REASON_AES
    if brake_avoidable is False:
        return REASON_INFEASIBLE
    if brake_avoidable is True:
        return REASON_BRAKE_OK
    return REASON_NO_CONFLICT


def with_friction(reason: str, limited: bool) -> str:
    if not limited:
        return reason
    return f"{reason}|{REASON_FRICTION}"


def reason_text(reason: str) -> str:
    parts = []
    for item in reason.split("|"):
        parts.append(REASON_TEXT.get(item, item))
    return "；".join(parts)
