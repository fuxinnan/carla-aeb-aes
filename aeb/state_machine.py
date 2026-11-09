"""分级 AEB。确认、迟滞、最短保持和丢目标保持都按配置计帧。"""

from __future__ import annotations

from aeb.base import Transition
from aeb.config import StateMachineConfig
from schemas import AEB_EXIT, AEB_FULL, AEB_IDLE, AEB_PARTIAL, AEB_WARN, ControlRequest


class StateMachineAeb:
    """IDLE、WARN、PARTIAL、FULL、EXIT。每次离开当前状态都带原因。"""

    name = "state_machine"
    version = "round5-0"

    def __init__(self, config: StateMachineConfig) -> None:
        self.config = config
        self.aeb_state = AEB_IDLE
        self.last_transition: Transition | None = None
        self._confirm = 0
        self._hold = 0
        self._lost = 0
        self._clear = 0

    def decide(self, ego, tracks, threat) -> ControlRequest:
        del tracks
        ttc = threat.ttc_s
        if ttc is not None and ttc <= self.config.ttc_partial_on_s:
            self._confirm += 1
        else:
            self._confirm = 0
        new_state, reason, decel = self._next(float(ego.speed_mps), ttc)
        self.last_transition = None
        if new_state != self.aeb_state:
            if not reason:
                raise RuntimeError("状态变化缺少原因")
            self.last_transition = Transition(self.aeb_state, new_state, reason)
            self.aeb_state = new_state
            self._hold = 1
            self._lost = 0
            self._clear = 0
        elif self.aeb_state != AEB_IDLE:
            self._hold += 1
        return ControlRequest(
            throttle=0.0,
            brake=0.0,
            steer=0.0,
            desired_speed_mps=None,
            target_decel_mps2=decel,
        )

    def _next(self, speed_mps: float, ttc: float | None) -> tuple[str, str, float]:
        cfg = self.config
        state = self.aeb_state
        braking = state in (AEB_PARTIAL, AEB_FULL)
        if braking and speed_mps <= cfg.stop_speed_mps:
            return AEB_EXIT, "stopped", 0.0
        if braking and ttc is None:
            return self._lost_target(state)
        if braking:
            self._lost = 0
        if state in (AEB_IDLE, AEB_WARN):
            return self._from_watching(state, ttc)
        if state == AEB_PARTIAL:
            return self._from_partial(ttc)
        if state == AEB_FULL:
            return self._from_full(ttc)
        if ttc is not None and ttc <= cfg.ttc_warn_on_s:
            return AEB_WARN, "rearmed", 0.0
        return AEB_EXIT, "", 0.0

    def _lost_target(self, state: str) -> tuple[str, str, float]:
        cfg = self.config
        self._lost += 1
        decel = self._decel(state)
        if self._lost <= cfg.lost_hold_frames:
            return state, "", decel
        if self._hold < cfg.min_hold_frames:
            return state, "", decel
        self._clear += 1
        if self._clear >= cfg.clear_frames:
            return AEB_EXIT, "target_lost", 0.0
        return state, "", decel

    def _from_watching(self, state: str, ttc: float | None) -> tuple[str, str, float]:
        cfg = self.config
        if ttc is None:
            if state == AEB_WARN:
                return AEB_IDLE, "threat_unavailable", 0.0
            return AEB_IDLE, "", 0.0
        if state == AEB_WARN and ttc > cfg.ttc_warn_off_s:
            return AEB_IDLE, "ttc_above_warn", 0.0
        if self._confirm >= cfg.confirm_frames and ttc <= cfg.ttc_full_on_s:
            return AEB_FULL, "ttc_below_full", cfg.full_decel_mps2
        if self._confirm >= cfg.confirm_frames and ttc <= cfg.ttc_partial_on_s:
            return AEB_PARTIAL, "ttc_below_partial", cfg.partial_decel_mps2
        if ttc <= cfg.ttc_warn_on_s:
            if state == AEB_WARN:
                return AEB_WARN, "", 0.0
            return AEB_WARN, "ttc_below_warn", 0.0
        return state, "", 0.0

    def _from_partial(self, ttc: float | None) -> tuple[str, str, float]:
        cfg = self.config
        if ttc is not None and ttc <= cfg.ttc_full_on_s:
            return AEB_FULL, "ttc_below_full", cfg.full_decel_mps2
        if ttc is not None and ttc <= cfg.ttc_partial_off_s:
            self._clear = 0
            return AEB_PARTIAL, "", cfg.partial_decel_mps2
        if self._hold < cfg.min_hold_frames:
            return AEB_PARTIAL, "", cfg.partial_decel_mps2
        self._clear += 1
        if self._clear >= cfg.clear_frames:
            return AEB_EXIT, "threat_cleared", 0.0
        return AEB_PARTIAL, "", cfg.partial_decel_mps2

    def _from_full(self, ttc: float | None) -> tuple[str, str, float]:
        cfg = self.config
        if ttc is not None and ttc <= cfg.ttc_full_off_s:
            self._clear = 0
            return AEB_FULL, "", cfg.full_decel_mps2
        if self._hold < cfg.min_hold_frames:
            return AEB_FULL, "", cfg.full_decel_mps2
        if ttc is not None and ttc <= cfg.ttc_partial_off_s:
            return AEB_PARTIAL, "ttc_above_full", cfg.partial_decel_mps2
        self._clear += 1
        if self._clear >= cfg.clear_frames:
            return AEB_EXIT, "threat_cleared", 0.0
        return AEB_FULL, "", cfg.full_decel_mps2

    def _decel(self, state: str) -> float:
        if state == AEB_FULL:
            return self.config.full_decel_mps2
        if state == AEB_PARTIAL:
            return self.config.partial_decel_mps2
        return 0.0
