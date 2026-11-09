"""把左右两条候选收成一份 AesPlan。两侧都不可行时不硬选。"""

from __future__ import annotations

from aes.check import inspect_candidate
from aes.config import AesConfig
from aes.trajectory import generate_trajectory
from aes.types import (
    DIRECTION_LEFT,
    DIRECTION_RIGHT,
    REASON_HIGHER_COST,
    REASON_TEXT,
    REASON_TIE,
    AesEgo,
    AesPlan,
    AesTarget,
    Candidate,
    DrivableCorridor,
    Rejection,
    direction_text,
)


def plan_avoidance(
    ego: AesEgo,
    targets: tuple[AesTarget, ...] | list[AesTarget],
    road,
    corridor: DrivableCorridor,
    config: AesConfig,
) -> AesPlan:
    """生成向左、向右两条轨迹并检查。选用的那条写进 points，其余都有拒绝原因。"""
    target_tuple = tuple(targets)
    left = _candidate(DIRECTION_LEFT, ego, target_tuple, road, corridor, config)
    right = _candidate(DIRECTION_RIGHT, ego, target_tuple, road, corridor, config)
    chosen, rejections = _select(left, right, config.constraints.cost_tie_epsilon)
    if chosen is None:
        return AesPlan(
            avoidable=False,
            direction=None,
            points=(),
            cost=None,
            rejections=tuple(rejections),
            candidates=(left, right),
            boundary_source=corridor.source,
        )
    return AesPlan(
        avoidable=True,
        direction=chosen.direction,
        points=chosen.points,
        cost=chosen.cost,
        rejections=tuple(rejections),
        candidates=(left, right),
        boundary_source=corridor.source,
    )


def _candidate(direction, ego, targets, road, corridor, config) -> Candidate:
    points = generate_trajectory(direction, ego, road, config)
    return inspect_candidate(direction, points, ego, targets, road, corridor, config)


def _select(left: Candidate, right: Candidate, tie_epsilon: float) -> tuple[Candidate | None, list[Rejection]]:
    feasible = [item for item in (left, right) if item.feasible]
    if not feasible:
        return None, [_rejection_from(left), _rejection_from(right)]
    if len(feasible) == 1:
        chosen = feasible[0]
        other = right if chosen.direction == DIRECTION_LEFT else left
        return chosen, [_rejection_from(other)]
    gap = left.cost - right.cost
    if abs(gap) <= tie_epsilon:
        return left, [
            Rejection(
                DIRECTION_RIGHT,
                (REASON_TIE,),
                f"{REASON_TEXT[REASON_TIE]}。{direction_text(DIRECTION_RIGHT)}同样可行",
            )
        ]
    if gap < 0.0:
        return left, [
            Rejection(
                DIRECTION_RIGHT,
                (REASON_HIGHER_COST,),
                f"{REASON_TEXT[REASON_HIGHER_COST]}。{direction_text(DIRECTION_RIGHT)}累计代价 {right.cost:.2f}，{direction_text(DIRECTION_LEFT)} {left.cost:.2f}",
            )
        ]
    return right, [
        Rejection(
            DIRECTION_LEFT,
            (REASON_HIGHER_COST,),
            f"{REASON_TEXT[REASON_HIGHER_COST]}。{direction_text(DIRECTION_LEFT)}累计代价 {left.cost:.2f}，{direction_text(DIRECTION_RIGHT)} {right.cost:.2f}",
        )
    ]


def _rejection_from(candidate: Candidate) -> Rejection:
    return Rejection(candidate.direction, candidate.reasons, candidate.text)
