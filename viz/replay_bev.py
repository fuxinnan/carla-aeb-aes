"""只读 frames.jsonl，用 matplotlib 做俯视回放，并按 env.json 选择 gif 或 mp4。"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
from pathlib import Path

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if not any(Path(entry).resolve() == _PROJECT_ROOT for entry in sys.path if entry):
    sys.path.insert(0, str(_PROJECT_ROOT))

import numpy as np

from scenarios.geometry import box_corners
from viz.common import _prepare_matplotlib, aeb_state_color


def choose_export_format(env_path: str | Path | None = None) -> str:
    """ffmpeg 可用就写 mp4，imageio 可用就写 gif，否则用 env.json 里可用的 OpenCV 写 mp4。"""
    path = Path(env_path) if env_path is not None else _PROJECT_ROOT / "env.json"
    viz = {}
    if path.is_file():
        viz = json.loads(path.read_text(encoding="utf-8")).get("viz", {})
    if viz.get("ffmpeg", {}).get("available"):
        return "ffmpeg"
    if viz.get("imageio", {}).get("available"):
        return "imageio"
    if viz.get("opencv-python", {}).get("available") or importlib.util.find_spec("cv2") is not None:
        return "opencv"
    if importlib.util.find_spec("PIL") is not None:
        return "pillow"
    raise RuntimeError("env.json 里没有可用的 gif/mp4 写出方式")


def export_bev_replay(run_dir: str | Path, env_path: str | Path | None = None) -> Path:
    """俯视回放。有目标轨迹和 target_trigger 事件时，一并画出来。"""
    root = Path(run_dir)
    frames = _read_frames(root / "frames.jsonl")
    if not frames:
        raise ValueError(f"{root} 的 frames.jsonl 是空的")
    trails = _target_trails(_read_jsonl(root / "targets.jsonl"))
    trigger_times = [
        float(row["sim_time_s"])
        for row in _read_jsonl(root / "events.jsonl")
        if row.get("event_type") == "target_trigger"
    ]
    backend = choose_export_format(env_path)
    scene_id = str(frames[0]["scene_id"])
    safe_scene = "".join(char if char not in '<>:"/\\|?*' else "_" for char in scene_id.strip()) or "scene"
    suffix = "gif" if backend in {"imageio", "pillow"} else "mp4"
    figures = root / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    output = figures / f"{safe_scene}_bev.{suffix}"
    ego_length_m, ego_width_m = _ego_size(root)
    _write_animation(frames, output, backend, trails, trigger_times, ego_length_m, ego_width_m)
    return output


def _read_frames(path: Path) -> list[dict]:
    if not path.is_file():
        raise FileNotFoundError(path)
    return _read_jsonl(path)


def _read_jsonl(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def _target_trails(rows: list[dict]) -> list[list[tuple[float, float, float]]]:
    grouped: dict[str, list[tuple[float, float, float]]] = {}
    for row in rows:
        track = row.get("track") or {}
        if "x_m" not in track or "y_m" not in track:
            continue
        track_id = str(track.get("track_id", "target"))
        grouped.setdefault(track_id, []).append(
            (float(row["sim_time_s"]), float(track["x_m"]), float(track["y_m"]))
        )
    return [sorted(points, key=lambda item: item[0]) for points in grouped.values()]


def _ego_size(root: Path) -> tuple[float | None, float | None]:
    manifest_path = root / "manifest.json"
    if not manifest_path.is_file():
        return None, None
    scenario = json.loads(manifest_path.read_text(encoding="utf-8")).get("scenario") or {}
    ego = scenario.get("ego") or {}
    length = ego.get("length_m")
    width = ego.get("width_m")
    if length is None or width is None:
        return None, None
    return float(length), float(width)


def _write_animation(
    frames: list[dict],
    output: Path,
    backend: str,
    trails: list[list[tuple[float, float, float]]] | None = None,
    trigger_times: list[float] | None = None,
    ego_length_m: float | None = None,
    ego_width_m: float | None = None,
) -> None:
    _prepare_matplotlib()
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    trails = trails or []
    trigger_times = trigger_times or []
    xs = np.array([float(row["ego"]["x_m"]) for row in frames], dtype=float)
    ys = np.array([float(row["ego"]["y_m"]) for row in frames], dtype=float)
    times = np.array([float(row["sim_time_s"]) for row in frames], dtype=float)
    extra_x = [point[1] for trail in trails for point in trail]
    extra_y = [point[2] for trail in trails for point in trail]
    fps = _playback_fps(times)
    xmin, xmax, ymin, ymax = _view_limits(list(xs) + extra_x, list(ys) + extra_y)
    figure, axis = plt.subplots(figsize=(5, 5), dpi=80)
    axis.set_xlim(xmin, xmax)
    axis.set_ylim(ymin, ymax)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("世界 X (m)")
    axis.set_ylabel("世界 Y (m)")
    axis.grid(True, alpha=0.3)
    axis.scatter([xs[0]], [ys[0]], color="#43A047", s=28, label="起点", zorder=3)
    trail, = axis.plot([], [], color="#1E88E5", linewidth=1.5, label="自车")
    point, = axis.plot([], [], "o", color="#E53935", markersize=5)
    target_lines = []
    target_points = []
    for trail_index, _samples in enumerate(trails):
        label = "目标" if trail_index == 0 else None
        target_line, = axis.plot([], [], color="#FB8C00", linewidth=1.5, label=label)
        target_point, = axis.plot([], [], "o", color="#FB8C00", markersize=4)
        target_lines.append(target_line)
        target_points.append(target_point)
    trigger_mark, = axis.plot([], [], linestyle="None", marker="*", color="#8E24AA", markersize=12, label="触发", zorder=5)
    ego_box = Polygon([(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)], closed=True, alpha=0.0, zorder=4)
    axis.add_patch(ego_box)
    trigger_at = trigger_times[0] if trigger_times else None
    trigger_xy = _trigger_marks(frames, trails, trigger_at)
    axis.legend(frameon=False, loc="upper right")

    def draw(index: int) -> None:
        trail.set_data(xs[: index + 1], ys[: index + 1])
        point.set_data([xs[index]], [ys[index]])
        point.set_color("#E53935")
        ego_box.set_alpha(0.0)
        ego = frames[index].get("ego") or {}
        state = frames[index].get("aeb_state")
        yaw = ego.get("yaw_rad")
        if state and yaw is not None and ego_length_m is not None and ego_width_m is not None:
            color = aeb_state_color(str(state))
            corners = box_corners(
                float(ego["x_m"]),
                float(ego["y_m"]),
                float(yaw),
                ego_length_m,
                ego_width_m,
            )
            ego_box.set_xy(corners)
            ego_box.set_facecolor(color)
            ego_box.set_edgecolor(color)
            ego_box.set_alpha(0.45)
            point.set_color(color)
        now = float(times[index])
        for samples, target_line, target_point in zip(trails, target_lines, target_points):
            taken = [item for item in samples if item[0] <= now + 1e-9]
            target_line.set_data([item[1] for item in taken], [item[2] for item in taken])
            if taken:
                target_point.set_data([taken[-1][1]], [taken[-1][2]])
            else:
                target_point.set_data([], [])
        if trigger_xy and trigger_at is not None and now + 1e-9 >= trigger_at:
            trigger_mark.set_data([item[0] for item in trigger_xy], [item[1] for item in trigger_xy])
        else:
            trigger_mark.set_data([], [])
        axis.set_title(f"t = {now:.2f} s")
        figure.canvas.draw()

    if backend == "opencv":
        _write_opencv(figure, draw, len(frames), output, fps)
    elif backend == "ffmpeg":
        _write_matplotlib(figure, draw, len(frames), output, fps, "ffmpeg")
    elif backend == "imageio":
        _write_imageio(figure, draw, len(frames), output, fps)
    else:
        _write_matplotlib(figure, draw, len(frames), output, fps, "pillow")
    plt.close(figure)


def _write_opencv(figure, draw, count: int, output: Path, fps: float) -> None:
    import cv2

    writer = None
    width = 0
    height = 0
    for index in range(count):
        draw(index)
        rgb = _canvas_rgb(figure)
        if writer is None:
            height, width = rgb.shape[:2]
            width -= width % 2
            height -= height % 2
            writer = cv2.VideoWriter(
                str(output),
                cv2.VideoWriter_fourcc(*"mp4v"),
                float(fps),
                (width, height),
            )
            if not writer.isOpened():
                raise RuntimeError(f"OpenCV 无法写出 {output}")
        rgb = rgb[:height, :width]
        if rgb.shape[0] != height or rgb.shape[1] != width:
            raise RuntimeError(f"回放画面尺寸变成 {rgb.shape[1]}x{rgb.shape[0]}，开头是 {width}x{height}")
        writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    if writer is not None:
        writer.release()


def _write_imageio(figure, draw, count: int, output: Path, fps: float) -> None:
    import imageio.v2 as imageio

    duration = 1.0 / fps
    with imageio.get_writer(output, format="GIF", mode="I", duration=duration) as writer:
        for index in range(count):
            draw(index)
            writer.append_data(_canvas_rgb(figure))


def _write_matplotlib(figure, draw, count: int, output: Path, fps: float, kind: str) -> None:
    from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter

    def update(index):
        draw(index)
        return ()

    animation = FuncAnimation(figure, update, frames=count, blit=False)
    if kind == "ffmpeg":
        animation.save(output, writer=FFMpegWriter(fps=fps))
    else:
        animation.save(output, writer=PillowWriter(fps=fps))


def _canvas_rgb(figure) -> np.ndarray:
    canvas = figure.canvas
    if hasattr(canvas, "buffer_rgba"):
        rgba = np.asarray(canvas.buffer_rgba())
        return np.ascontiguousarray(rgba[:, :, :3])
    width, height = canvas.get_width_height()
    buffer = np.frombuffer(canvas.tostring_rgb(), dtype=np.uint8)
    return buffer.reshape((height, width, 3))


def _view_limits(xs: list[float], ys: list[float]) -> tuple[float, float, float, float]:
    pad = 5.0
    xmin = min(xs) - pad
    xmax = max(xs) + pad
    ymin = min(ys) - pad
    ymax = max(ys) + pad
    if xmax - xmin < 10.0:
        center = (xmin + xmax) * 0.5
        xmin, xmax = center - 5.0, center + 5.0
    if ymax - ymin < 10.0:
        center = (ymin + ymax) * 0.5
        ymin, ymax = center - 5.0, center + 5.0
    return xmin, xmax, ymin, ymax


def _trigger_marks(frames, trails, trigger_at: float | None) -> list[tuple[float, float]]:
    if trigger_at is None:
        return []
    marks = [_nearest_frame(frames, trigger_at)]
    for trail in trails:
        marks.append(_nearest_trail(trail, trigger_at))
    return marks


def _nearest_frame(frames, sim_time_s: float) -> tuple[float, float]:
    row = min(frames, key=lambda item: abs(float(item["sim_time_s"]) - sim_time_s))
    return float(row["ego"]["x_m"]), float(row["ego"]["y_m"])


def _nearest_trail(trail, sim_time_s: float) -> tuple[float, float]:
    item = min(trail, key=lambda point: abs(point[0] - sim_time_s))
    return item[1], item[2]


def _playback_fps(times: np.ndarray) -> float:
    if len(times) < 2:
        return 20.0
    step = float(times[-1] - times[0]) / float(len(times) - 1)
    if step <= 1e-6:
        return 20.0
    return float(min(60.0, max(1.0, round(1.0 / step))))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="从 frames.jsonl 导出俯视回放。")
    parser.add_argument("run_dir", help="运行目录，只读取其中的 frames.jsonl")
    args = parser.parse_args(argv)
    path = export_bev_replay(args.run_dir)
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
