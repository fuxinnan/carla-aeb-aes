"""生成一次运行的清单。只记录版本、种子和配置，不连接仿真器。"""

from __future__ import annotations

import hashlib
import json
import platform
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Mapping

import yaml

from schemas import SCHEMA_VERSION, to_plain


class _DetectGit:
    """调用方没传 code_version 时，用这个标记去读 git。"""


_DETECT_GIT = _DetectGit()

_SUBSTEP_KEYS = (
    "substepping",
    "max_substep_delta_time_s",
    "max_substeps",
)


def file_sha256(path: Path) -> str:
    """配置文件原始字节的 SHA-256，不改写文件。"""
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_head(cwd: Path | None = None) -> str | None:
    """项目根目录自己的 git HEAD。

    目录不是仓库、没有 git，或只是落在上层仓库里面时，都返回 None。
    这样不会把旁边 CARLA 源码仓库的提交号当成这个项目的代码版本。
    """
    root = Path(cwd or Path(__file__).resolve().parent).resolve()
    try:
        toplevel = subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            cwd=str(root),
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=str(root),
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if toplevel.returncode != 0 or completed.returncode != 0:
        return None
    if Path(toplevel.stdout.strip()).resolve() != root:
        return None
    commit = completed.stdout.strip()
    return commit or None


def load_strategy_config(path: Path) -> dict:
    """读取策略配置。支持 JSON 和 YAML，结果必须是对象。"""
    raw = path.read_bytes()
    text = raw.decode("utf-8")
    suffix = path.suffix.lower()
    if suffix in {".yaml", ".yml"}:
        loaded = yaml.safe_load(text)
    elif suffix == ".json":
        loaded = json.loads(text)
    else:
        raise ValueError(f"策略配置只接受 .json / .yaml / .yml，收到 {path.name}")
    if not isinstance(loaded, dict):
        raise ValueError(f"策略配置必须是对象: {path}")
    return to_plain(loaded)


def _substep(physics_substep: Mapping | None) -> dict:
    source = dict(physics_substep or {})
    unknown = sorted(set(source) - set(_SUBSTEP_KEYS))
    if unknown:
        raise ValueError(f"physics_substep 含未知字段: {unknown}")
    return {key: source.get(key) for key in _SUBSTEP_KEYS}


def build_manifest(
    *,
    run_id: str,
    scene_id: str,
    strategy_name: str,
    strategy_version: str,
    strategy_config_path: str | Path,
    carla_version: str | None = None,
    map_name: str | None = None,
    fixed_delta_seconds: float | None = None,
    synchronous_mode: bool | None = None,
    physics_substep: Mapping | None = None,
    python_seed: int | None = None,
    numpy_seed: int | None = None,
    traffic_manager_seed: int | None = None,
    weather: Mapping | None = None,
    code_version: str | None | _DetectGit = _DETECT_GIT,
) -> dict:
    """组装 manifest 字典。CARLA 版本由调用方传入，这里不导入 carla。

    physics_substep 只接受三个键：substepping、max_substep_delta_time_s（秒）、max_substeps。
    weather 是普通字典，原样写入。code_version 省略时读 git，显式传 None 就写 null。
    """
    config_path = Path(strategy_config_path)
    if isinstance(code_version, _DetectGit):
        resolved_code_version = git_head(Path(__file__).resolve().parent)
    else:
        resolved_code_version = code_version

    weather_plain = None if weather is None else to_plain(dict(weather))
    return {
        "schema_version": SCHEMA_VERSION,
        "run_id": run_id,
        "scene_id": scene_id,
        "strategy_name": strategy_name,
        "strategy_version": strategy_version,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "carla_version": carla_version,
        "python_version": platform.python_version(),
        "map": map_name,
        "fixed_delta_seconds": fixed_delta_seconds,
        "synchronous_mode": synchronous_mode,
        "physics_substep": _substep(physics_substep),
        "seeds": {
            "python": python_seed,
            "numpy": numpy_seed,
            "traffic_manager": traffic_manager_seed,
        },
        "weather": weather_plain,
        "strategy_config": load_strategy_config(config_path),
        "strategy_config_path": str(config_path),
        "strategy_config_sha256": file_sha256(config_path),
        "code_version": resolved_code_version,
    }
