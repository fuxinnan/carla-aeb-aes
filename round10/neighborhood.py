"""在难例的速度和距离附近加密。只写到 tuning 目录，出现时机记在采样说明里。"""

from __future__ import annotations

import json
from pathlib import Path

import yaml

from round10.guard import assert_tuning_payload, refuse_test_write


def neighborhood_points(speed: float, distance: float) -> list[tuple[float, float]]:
    """速度 ±1 m/s，距离 ±2 m。非正的距离丢掉。"""
    points = []
    for speed_step in (-1.0, 0.0, 1.0):
        for distance_step in (-2.0, 0.0, 2.0):
            new_speed = speed + speed_step
            new_distance = distance + distance_step
            if new_speed <= 0.0 or new_distance <= 0.0:
                continue
            points.append((new_speed, new_distance))
    return points


def write_neighborhood(cases: list[dict], directory: str | Path) -> list[dict]:
    """每个选中的难例写一组场景。目录必须还能被第 9 轮的 load_split 读。"""
    root = Path(directory)
    refuse_test_write(root)
    if root.name != "tuning":
        raise ValueError(f"邻域场景要放在名为 tuning 的目录里: {root}")
    root.mkdir(parents=True, exist_ok=True)
    written = []
    seed = 400
    for case in cases:
        template = case["manifest"]["template"]
        for speed, distance in neighborhood_points(case["ego_speed_mps"], case["distance_m"]):
            instance = f"v{int(speed)}_d{int(distance)}"
            payload = {
                "template": template,
                "instance": instance,
                "seed": seed,
                "split": "tuning",
                "ego": {"speed_mps": speed},
                "distance_m": distance,
                "end": {"hard_negative": template == "parallel_vehicle"},
            }
            assert_tuning_payload(payload)
            path = root / f"{template}_{instance}.yaml"
            refuse_test_write(path)
            path.write_text(yaml.safe_dump(payload, allow_unicode=True, sort_keys=False), encoding="utf-8")
            written.append(
                {
                    "path": str(path),
                    "template": template,
                    "ego_speed_mps": speed,
                    "distance_m": distance,
                    "source_scene_id": case["scene_id"],
                    "appearance_s": case.get("appearance_s"),
                }
            )
            seed += 1
    note = {
        "appearance_s": "取源日志里第一次有效 TTC 的时间。现有场景文件只有速度和距离，加密不新增字段，第 9 轮加载器不用改。",
        "scenes": written,
    }
    note_path = root.parent / "sampling.json"
    refuse_test_write(note_path)
    note_path.write_text(json.dumps(note, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return written
