"""威胁评估。本包不连接仿真器，也不做制动或转向决策。"""

from threat.assessor import PREDICTION, REASON_TEXT, ThreatAssessor
from threat.pipeline import assess_frame

__all__ = [
    "PREDICTION",
    "REASON_TEXT",
    "ThreatAssessor",
    "assess_frame",
]
