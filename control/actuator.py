"""把目标减速度映射成制动请求。线性、限幅、限斜率，两种 AEB 共用。"""

from __future__ import annotations

from dataclasses import replace

from aeb.config import ActuatorConfig
from schemas import ActualControl, ControlRequest


class BrakeActuator:
    """applied 是斜率限制后的减速度。brake 是它的线性映射，范围 0~max_brake。"""

    def __init__(self, config: ActuatorConfig, dt_s: float) -> None:
        if dt_s <= 0.0:
            raise ValueError("执行层时步必须大于 0")
        self.config = config
        self.dt_s = float(dt_s)
        self.applied = 0.0

    def command(self, request: ControlRequest) -> tuple[ControlRequest, ActualControl]:
        raw = 0.0 if request.target_decel_mps2 is None else float(request.target_decel_mps2)
        if raw < 0.0:
            raw = 0.0
        limited = min(raw, self.config.max_decel_mps2)
        step = self.config.slew_decel_mps3 * self.dt_s
        delta = limited - self.applied
        if delta > step:
            delta = step
        elif delta < -step:
            delta = -step
        self.applied = min(self.config.max_decel_mps2, max(0.0, self.applied + delta))
        asked = replace(request, brake=self._brake(limited), target_decel_mps2=raw)
        actual = ActualControl(
            throttle=float(request.throttle),
            brake=self._brake(self.applied),
            steer=float(request.steer),
            desired_speed_mps=request.desired_speed_mps,
            applied_decel_mps2=self.applied,
        )
        return asked, actual

    def _brake(self, decel_mps2: float) -> float:
        return min(self.config.max_brake, max(0.0, decel_mps2 * self.config.brake_per_decel))
