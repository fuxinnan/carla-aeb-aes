"""固定 TTC 的 AEB。低于阈值制动，不低于就松开。不看上一帧的制动请求。"""

from __future__ import annotations

from aeb.base import Transition
from aeb.config import FixedTtcConfig
from schemas import AEB_FULL, AEB_IDLE, ControlRequest


class FixedTtcAeb:
    """制动量只由这一帧的 TTC 决定。上一状态只用来记转移，不改变请求。"""

    name = "fixed_ttc"
    version = "round5-0"

    def __init__(self, config: FixedTtcConfig) -> None:
        self.config = config
        self.aeb_state = AEB_IDLE
        self.last_transition: Transition | None = None

    def decide(self, ego, tracks, threat) -> ControlRequest:
        del ego, tracks
        ttc = threat.ttc_s
        braking = ttc is not None and ttc < self.config.ttc_brake_s
        new_state = AEB_FULL if braking else AEB_IDLE
        decel = self.config.target_decel_mps2 if braking else 0.0
        self.last_transition = None
        if new_state != self.aeb_state:
            if braking:
                reason = "ttc_below_threshold"
            elif ttc is None:
                reason = "ttc_unavailable"
            else:
                reason = "ttc_above_threshold"
            self.last_transition = Transition(self.aeb_state, new_state, reason)
            self.aeb_state = new_state
        return ControlRequest(
            throttle=0.0,
            brake=0.0,
            steer=0.0,
            desired_speed_mps=None,
            target_decel_mps2=decel,
        )
