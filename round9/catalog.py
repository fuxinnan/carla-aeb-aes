"""第 9 轮场景网格。调参目录和测试目录的数字不重叠，按写死的种子划分。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


TEMPLATES = (
    "vehicle_cut_in",
    "occlusion_reveal",
    "left_avoidable",
    "right_avoidable",
    "both_unavoidable",
    "parallel_vehicle",
)
TUNING_SPEEDS = (8.0, 11.0)
TUNING_DISTANCES = (16.0, 26.0)
TEST_SPEEDS = (10.0, 14.0)
TEST_DISTANCES = (18.0, 32.0)
EGO_S_M = 8.0
DT_S = 0.05
TIMEOUT_S = 8.0


@dataclass(frozen=True)
class GridScene:
    template: str
    instance: str
    seed: int
    split: str
    ego_speed_mps: float
    distance_m: float
    hard_negative: bool
    path: Path | None = None

    @property
    def scene_id(self) -> str:
        return f"{self.template}_{self.instance}"


def write_grid(root: str | Path) -> tuple[list[GridScene], list[GridScene]]:
    """把网格写成 YAML。之后只读这些文件。"""
    base = Path(root)
    tuning = [_scene(template, speed, distance, "tuning", 20 + index) for index, (template, speed, distance) in enumerate(_pairs(TUNING_SPEEDS, TUNING_DISTANCES))]
    test = [_scene(template, speed, distance, "test", 80 + index) for index, (template, speed, distance) in enumerate(_pairs(TEST_SPEEDS, TEST_DISTANCES))]
    _dump(base / "tuning", tuning)
    _dump(base / "test", test)
    return tuning, test


def load_scene(path: str | Path) -> GridScene:
    """读一条场景 YAML。字段和网格文件相同。"""
    return _read(Path(path))


def load_split(directory: str | Path, expected_split: str) -> list[GridScene]:
    """只读一个划分目录。目录名必须和场景里的 split 一致。"""
    root = Path(directory)
    if root.name != expected_split:
        raise ValueError(f"目录名必须是 {expected_split}: {root}")
    scenes = []
    for path in sorted(root.glob("*.yaml")):
        scene = _read(path)
        if scene.split != expected_split:
            raise ValueError(f"{path} 的 split 是 {scene.split}，不能放进 {expected_split}")
        scenes.append(scene)
    if not scenes:
        raise ValueError(f"{root} 里没有场景")
    return scenes


def _pairs(speeds, distances):
    for template in TEMPLATES:
        for speed in speeds:
            for distance in distances:
                yield template, speed, distance


def _scene(template: str, speed: float, distance: float, split: str, seed: int) -> GridScene:
    speed_token = str(int(speed))
    distance_token = str(int(distance))
    return GridScene(
        template=template,
        instance=f"v{speed_token}_d{distance_token}",
        seed=seed,
        split=split,
        ego_speed_mps=speed,
        distance_m=distance,
        hard_negative=template == "parallel_vehicle",
    )


def _dump(directory: Path, scenes: list[GridScene]) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for scene in scenes:
        path = directory / f"{scene.scene_id}.yaml"
        payload = {
            "template": scene.template,
            "instance": scene.instance,
            "seed": scene.seed,
            "split": scene.split,
            "ego": {"speed_mps": scene.ego_speed_mps},
            "distance_m": scene.distance_m,
            "end": {"hard_negative": scene.hard_negative},
        }
        path.write_text(yaml.safe_dump(payload, allow_unicode=True, sort_keys=False), encoding="utf-8")


def _read(path: Path) -> GridScene:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"{path} 必须是映射")
    unknown = set(raw) - {"template", "instance", "seed", "split", "ego", "distance_m", "end"}
    if unknown:
        raise ValueError(f"{path} 含未知字段: {sorted(unknown)}")
    template = str(raw["template"])
    if template not in TEMPLATES:
        raise ValueError(f"未知模板 {template}")
    split = str(raw["split"])
    if split not in {"tuning", "test"}:
        raise ValueError(f"split 必须是 tuning 或 test: {path}")
    return GridScene(
        template=template,
        instance=str(raw["instance"]),
        seed=int(raw["seed"]),
        split=split,
        ego_speed_mps=float(raw["ego"]["speed_mps"]),
        distance_m=float(raw["distance_m"]),
        hard_negative=bool(raw["end"]["hard_negative"]),
        path=path,
    )
