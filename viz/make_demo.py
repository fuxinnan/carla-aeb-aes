"""从已有日志合成五段俯视演示。不改策略，也不连接仿真器。"""

from __future__ import annotations

import json
from pathlib import Path

from scenarios.geometry import box_corners
from viz.common import _prepare_matplotlib
from viz.replay_bev import _canvas_rgb


ROOT = Path(__file__).resolve().parents[1]
VIDEO_DIR = ROOT / "videos"
FPS = 20.0

STRATEGY_TEXT = {
    "no_intervention": "无干预",
    "fixed_ttc": "固定 TTC",
    "state_machine": "状态机 AEB",
    "aeb_aes": "状态机 AEB+AES",
}

SEGMENTS = (
    {
        "id": "01_no_intervention_collision",
        "title": "无干预发生碰撞",
        "run": "runs/round6/test/pedestrian_crossing_collision/no_intervention/rep_00",
        "cause": None,
    },
    {
        "id": "02_fixed_ttc_hard_negative",
        "title": "固定阈值 AEB，硬负例",
        "run": "runs/round6/test/pedestrian_crossing_clear_pass/fixed_ttc/rep_00",
        "cause": None,
        "note": "日志未误刹",
    },
    {
        "id": "03_state_machine_avoid",
        "title": "状态机 AEB 避开碰撞",
        "run": "runs/round6/tuning/pedestrian_crossing_collision/state_machine/rep_00",
        "cause": None,
    },
    {
        "id": "04_aeb_aes_avoid",
        "title": "状态机加 AES 避开碰撞",
        "run": "runs/round9/logs/test/aeb_aes/left_avoidable_v14_d18/rep_00",
        "cause": None,
    },
    {
        "id": "05_late_brake_failure",
        "title": "失败案例",
        "run": "runs/round9/logs/search/state_c3_p1_4/left_avoidable_v11_d16/rep_00",
        "cause": "晚制动",
    },
)


def main() -> None:
    camera = _camera_available()
    if camera:
        raise RuntimeError("env.json 标了摄像头，但这一版演示没有第三人称导出")
    VIDEO_DIR.mkdir(parents=True, exist_ok=True)
    catalog = []
    for segment in SEGMENTS:
        catalog.append(_render(segment))
    payload = {
        "camera": "bev_only",
        "camera_reason": "env.json 没有摄像头字段，CARLA 服务端当时也没连接，只做俯视。",
        "segments": catalog,
    }
    (VIDEO_DIR / "manifest.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"写出 {len(catalog)} 段到 {VIDEO_DIR}", flush=True)


def _camera_available() -> bool:
    env_path = ROOT / "env.json"
    if not env_path.is_file():
        return False
    env = json.loads(env_path.read_text(encoding="utf-8"))
    camera = env.get("camera") or env.get("cameras")
    if isinstance(camera, dict):
        return bool(camera.get("available"))
    return bool(camera)


def _render(segment: dict) -> dict:
    run_dir = ROOT / segment["run"]
    frames = _read_jsonl(run_dir / "frames.jsonl")
    events = _read_jsonl(run_dir / "events.jsonl")
    targets = _tracks(_read_jsonl(run_dir / "targets.jsonl"))
    manifest = json.loads((run_dir / "manifest.json").read_text(encoding="utf-8"))
    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    length_m, width_m = _ego_size(manifest)
    subtitles = [_subtitle(frame, segment, summary) for frame in frames]
    _check_states(segment["id"], frames, events, subtitles)
    video_path = VIDEO_DIR / f"{segment['id']}.mp4"
    cover_path = VIDEO_DIR / f"{segment['id']}_cover.png"
    _write_video(frames, targets, subtitles, length_m, width_m, video_path, cover_path, _cover_index(frames, summary))
    return {
        "id": segment["id"],
        "title": segment["title"],
        "scene_id": manifest.get("scene_id") or summary.get("scene_id"),
        "strategy": manifest.get("strategy_name") or summary.get("strategy_name"),
        "run_dir": str(run_dir),
        "video": str(video_path.relative_to(ROOT)).replace("\\", "/"),
        "cover": str(cover_path.relative_to(ROOT)).replace("\\", "/"),
        "cause": segment["cause"],
        "collided": bool(summary.get("collided")),
    }


def _subtitle(frame: dict, segment: dict, summary: dict) -> str:
    threat = frame.get("threat") or {}
    ttc = threat.get("ttc_s")
    ttc_text = "—" if ttc is None else f"{float(ttc):.2f} s"
    speed = float((frame.get("ego") or {}).get("speed_mps") or 0.0)
    strategy = STRATEGY_TEXT.get(str(summary.get("strategy_name")), str(summary.get("strategy_name")))
    lines = [
        segment["title"],
        f"策略 {strategy}",
        f"状态 {frame.get('aeb_state')}",
        f"TTC {ttc_text}",
        f"速度 {speed:.1f} m/s",
    ]
    if segment["cause"]:
        lines.append(f"线索 {segment['cause']}")
    if segment.get("note"):
        lines.append(str(segment["note"]))
    return "\n".join(lines)


def _check_states(name: str, frames: list[dict], events: list[dict], subtitles: list[str]) -> None:
    switches = [event for event in events if event.get("event_type") == "aeb_state"]
    if not switches:
        idle = all(frame.get("aeb_state") == "IDLE" for frame in frames)
        text = all("状态 IDLE" in subtitle for subtitle in subtitles)
        if not idle or not text:
            raise RuntimeError(f"{name} 没有状态事件，但帧里的状态不是 IDLE")
        print(f"{name} 无状态切换，字幕全程 IDLE", flush=True)
        return
    for event in switches:
        moment = float(event["sim_time_s"])
        index = min(range(len(frames)), key=lambda item: abs(float(frames[item]["sim_time_s"]) - moment))
        destination = (event.get("detail") or {}).get("to")
        if frames[index].get("aeb_state") != destination:
            raise RuntimeError(f"{name} 在 {moment:.2f} s 的字幕状态和事件不一致")
        if f"状态 {destination}" not in subtitles[index]:
            raise RuntimeError(f"{name} 字幕没有写上 {destination}")
        print(f"{name} {moment:.2f} s -> {destination} 与 events.jsonl 一致", flush=True)


def _write_video(frames, targets, subtitles, length_m, width_m, video_path: Path, cover_path: Path, cover_index: int) -> None:
    import cv2

    _prepare_matplotlib()
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    figure, axis = plt.subplots(figsize=(8.0, 4.8), dpi=100)
    figure.subplots_adjust(top=0.72)
    writer = None
    cover = None
    for index, frame in enumerate(frames):
        axis.clear()
        for text in list(figure.texts):
            text.remove()
        ego = frame["ego"]
        now = float(frame["sim_time_s"])
        xmin, xmax, ymin, ymax = _window((float(ego["x_m"]), float(ego["y_m"])), _targets_now(targets, now))
        axis.set_xlim(xmin, xmax)
        axis.set_ylim(ymin, ymax)
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, linewidth=0.3)
        axis.plot(
            [float(item["ego"]["x_m"]) for item in frames[: index + 1]],
            [float(item["ego"]["y_m"]) for item in frames[: index + 1]],
            color="#1565C0",
            linewidth=1.2,
        )
        corners = box_corners(float(ego["x_m"]), float(ego["y_m"]), float(ego["yaw_rad"]), length_m, width_m)
        axis.add_patch(Polygon(corners, closed=True, facecolor="#1565C0", edgecolor="#0D47A1", alpha=0.85))
        for trail in targets.values():
            taken = [item for item in trail if item[0] <= now + 1e-9]
            if not taken:
                continue
            _when, x_m, y_m, yaw, box_length, box_width = taken[-1]
            if yaw is None or box_length is None or box_width is None:
                axis.plot([x_m], [y_m], "o", color="#C62828", markersize=4)
            else:
                box = box_corners(x_m, y_m, yaw, box_length, box_width)
                axis.add_patch(Polygon(box, closed=True, facecolor="#EF9A9A", edgecolor="#C62828", alpha=0.9))
        figure.text(0.02, 0.98, subtitles[index], va="top", ha="left", fontsize=10, bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.92, "edgecolor": "#B0BEC5"})
        axis.set_xlabel("X (m)")
        axis.set_ylabel("Y (m)")
        figure.canvas.draw()
        rgb = _canvas_rgb(figure)
        if writer is None:
            height, width = rgb.shape[:2]
            width -= width % 2
            height -= height % 2
            writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (width, height))
            if not writer.isOpened():
                raise RuntimeError(f"OpenCV 无法写出 {video_path}")
        bgr = cv2.cvtColor(rgb[:height, :width], cv2.COLOR_RGB2BGR)
        writer.write(bgr)
        if index == cover_index:
            cover = bgr.copy()
    if writer is not None:
        writer.release()
    plt.close(figure)
    if cover is None:
        raise RuntimeError(f"{video_path} 没有封面帧")
    ok, encoded = cv2.imencode(".png", cover)
    if not ok:
        raise RuntimeError(f"封面编码失败 {cover_path}")
    encoded.tofile(str(cover_path))


def _cover_index(frames: list[dict], summary: dict) -> int:
    if summary.get("collided"):
        return len(frames) - 1
    lateral = [abs(float(frame["ego"]["y_m"]) - float(frames[0]["ego"]["y_m"])) for frame in frames]
    if max(lateral) > 0.5:
        return max(range(len(frames)), key=lateral.__getitem__)
    braking = [index for index, frame in enumerate(frames) if frame.get("aeb_state") in {"PARTIAL", "FULL"}]
    if braking:
        return braking[len(braking) // 2]
    return len(frames) // 2


def _tracks(rows: list[dict]) -> dict[str, list[tuple]]:
    grouped: dict[str, list[tuple]] = {}
    for row in rows:
        track = row.get("track") or {}
        if "x_m" not in track:
            continue
        grouped.setdefault(str(track.get("track_id", "target")), []).append(
            (
                float(row["sim_time_s"]),
                float(track["x_m"]),
                float(track["y_m"]),
                None if track.get("yaw_rad") is None else float(track["yaw_rad"]),
                None if track.get("length_m") is None else float(track["length_m"]),
                None if track.get("width_m") is None else float(track["width_m"]),
            )
        )
    for points in grouped.values():
        points.sort(key=lambda item: item[0])
    return grouped


def _ego_size(manifest: dict) -> tuple[float, float]:
    ego = (manifest.get("scenario") or {}).get("ego") or {}
    length = ego.get("length_m")
    width = ego.get("width_m")
    if length is None or width is None:
        return 4.90, 2.12
    return float(length), float(width)


def _window(ego: tuple[float, float], targets: list[tuple[float, float]]) -> tuple[float, float, float, float]:
    xs = [ego[0]]
    ys = [ego[1]]
    for x_m, y_m in targets:
        if abs(x_m - ego[0]) < 40.0 and abs(y_m - ego[1]) < 40.0:
            xs.append(x_m)
            ys.append(y_m)
    xmin, xmax = min(xs) - 8.0, max(xs) + 8.0
    ymin, ymax = min(ys) - 8.0, max(ys) + 8.0
    if xmax - xmin < 28.0:
        center = (xmin + xmax) * 0.5
        xmin, xmax = center - 14.0, center + 14.0
    if ymax - ymin < 20.0:
        center = (ymin + ymax) * 0.5
        ymin, ymax = center - 10.0, center + 10.0
    return xmin, xmax, ymin, ymax


def _targets_now(targets: dict[str, list[tuple]], now: float) -> list[tuple[float, float]]:
    points = []
    for trail in targets.values():
        taken = [item for item in trail if item[0] <= now + 1e-9]
        if taken:
            points.append((taken[-1][1], taken[-1][2]))
    return points


def _read_jsonl(path: Path) -> list[dict]:
    if not path.is_file():
        raise FileNotFoundError(path)
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


if __name__ == "__main__":
    main()
