"""无干预基线。不制动、不转向，只保持场景里的正常行驶速度。"""

from __future__ import annotations

from schemas import AEB_IDLE, ControlRequest


class NoIntervention:
    """用来回答场景本身危不危险。这里不产生任何制动或转向请求。"""

    name = "no_intervention"
    version = "round4-0"

    def __init__(self, cruise_speed_mps: float | None = None) -> None:
        self.cruise_speed_mps = cruise_speed_mps
        self.aeb_state = AEB_IDLE
        self.last_transition = None

    def decide(self, ego, tracks, threat) -> ControlRequest:
        """油门、制动、转向都是 0。期望速度是构造时给定的巡航速度，单位 m/s。"""
        del ego, tracks, threat
        self.aeb_state = AEB_IDLE
        self.last_transition = None
        speed = None if self.cruise_speed_mps is None else float(self.cruise_speed_mps)
        return ControlRequest(
            throttle=0.0,
            brake=0.0,
            steer=0.0,
            desired_speed_mps=speed,
            target_decel_mps2=0.0,
        )
