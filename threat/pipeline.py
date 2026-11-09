"""把真值航迹、可见性和威胁评估串成一帧。不连接仿真器。"""

from __future__ import annotations

from perception.tracker import BaseTracker, TargetTruth
from perception.visibility import VisibilityModel
from schemas import ControlRequest, EgoState, ThreatResult, Track
from strategies.no_intervention import NoIntervention
from threat.assessor import ThreatAssessor


def assess_frame(
    *,
    timestamp_s: float,
    ego: EgoState,
    observations: list[TargetTruth],
    ego_length_m: float,
    ego_width_m: float,
    cruise_speed_mps: float | None,
    tracker: BaseTracker,
    visibility: VisibilityModel,
    assessor: ThreatAssessor,
    policy: NoIntervention,
) -> tuple[list[Track], ThreatResult, ControlRequest]:
    """一帧的航迹、主威胁和策略请求。巡航速度由策略自己保存，这里的参数只留给旧调用。"""
    tracks = visibility.apply(
        tracker.update(
            timestamp_s,
            ego,
            observations,
            ego_length_m=ego_length_m,
            ego_width_m=ego_width_m,
        ),
        ego,
        timestamp_s,
    )
    threat = assessor.assess(ego, tracks, ego_length_m=ego_length_m, ego_width_m=ego_width_m)
    del cruise_speed_mps
    return tracks, threat, policy.decide(ego, tracks, threat)
