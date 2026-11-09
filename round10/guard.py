"""测试集只读。挖掘和回归往那里写文件时直接拒绝。"""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PROTECTED = (
    ROOT / "configs" / "scenarios" / "test",
    ROOT / "configs" / "scenarios" / "round9" / "test",
)


def refuse_test_write(path: str | Path) -> None:
    """路径落在测试集目录里就报错。调用方还没写文件。"""
    resolved = Path(path).resolve()
    for protected in PROTECTED:
        root = protected.resolve()
        if resolved == root or root in resolved.parents:
            raise PermissionError(f"测试集只读，拒绝写入 {resolved}")


def assert_tuning_payload(payload: dict) -> None:
    if payload.get("split") != "tuning":
        raise PermissionError("新场景只能标成 tuning，不能标成 test")
