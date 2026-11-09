"""第 5 轮的两种 AEB。第 8 轮起可以构造状态机 AEB 加 AES。"""

from aeb.config import AebConfig, load_aeb_config
from aeb.fixed_ttc import FixedTtcAeb
from aeb.state_machine import StateMachineAeb
from strategies.no_intervention import NoIntervention

STRATEGY_NAMES = ("no_intervention", "fixed_ttc", "state_machine")


def build_policy(name: str, config: AebConfig, cruise_speed_mps: float | None, aes_config=None, round8_config=None):
    """按名字构造策略。巡航速度只给无干预基线使用。aeb_aes 额外使用 AES 和第 8 轮配置。"""
    if name == "no_intervention":
        return NoIntervention(cruise_speed_mps)
    if name == "fixed_ttc":
        return FixedTtcAeb(config.fixed_ttc)
    if name == "state_machine":
        return StateMachineAeb(config.state_machine)
    if name == "aeb_aes":
        from strategies.aeb_aes import AebAesPolicy

        return AebAesPolicy(config, aes_config, round8_config)
    raise ValueError(f"未知策略: {name}")


__all__ = [
    "AebConfig",
    "STRATEGY_NAMES",
    "FixedTtcAeb",
    "NoIntervention",
    "StateMachineAeb",
    "build_policy",
    "load_aeb_config",
]
