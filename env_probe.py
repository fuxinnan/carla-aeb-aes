"""Read-only environment probe for the CARLA AEB/AES closed-loop project.

Writes env.json next to this file. Uses the standard library and packages that
are already installed. Does not install anything and does not change the simulator.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.metadata
import json
import platform
import shutil
import socket
import subprocess
import sys
from pathlib import Path


HOST_DEFAULT = "localhost"
PORT_DEFAULT = 2000
TIMEOUT_DEFAULT = 3.0

# Attributes used to describe the running synchronous-mode setup.
# Missing attributes are skipped so older CARLA builds do not raise.
SYNC_SETTING_FIELDS = (
    "synchronous_mode",
    "fixed_delta_seconds",
    "no_rendering_mode",
    "substepping",
    "max_substep_delta_time",
    "max_substeps",
    "tile_stream_distance",
    "actor_active_distance",
    "spectator_as_ego",
    "deterministic_ragdolls",
    "max_culling_distance",
)


def _exception_text(exc: BaseException) -> str:
    return f"{type(exc).__name__}: {exc}"


def _distribution_version(dist_names: tuple[str, ...]) -> tuple[str | None, str | None]:
    """Return (version, distribution name) for the first installed distribution."""
    for name in dist_names:
        try:
            return importlib.metadata.version(name), name
        except importlib.metadata.PackageNotFoundError:
            continue
    return None, None


def _probe_import(dist_names: tuple[str, ...], import_name: str) -> dict:
    version, distribution = _distribution_version(dist_names)
    reason = None
    try:
        module = importlib.import_module(import_name)
    except Exception as exc:
        reason = _exception_text(exc)
        available = False
    else:
        available = True
        if version is None:
            module_version = getattr(module, "__version__", None)
            if module_version:
                version = str(module_version)
    return {
        "available": available,
        "version": version,
        "distribution": distribution,
        "reason": reason,
    }


def _json_safe(value):
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        if value != value or value in (float("inf"), float("-inf")):
            return None
        return value
    return str(value)


def _probe_os() -> dict:
    return {
        "system": platform.system(),
        "release": platform.release(),
        "version": platform.version(),
        "machine": platform.machine(),
        "platform": platform.platform(),
    }


def _probe_python() -> dict:
    return {
        "version": platform.python_version(),
        "executable": sys.executable,
        "implementation": platform.python_implementation(),
    }


def _nearby_carla_wheels() -> list[str]:
    """Names of CARLA wheels shipped beside this project. Nothing is installed."""
    dist = Path(__file__).resolve().parents[1] / "PythonAPI" / "carla" / "dist"
    if not dist.is_dir():
        return []
    return sorted(path.name for path in dist.glob("carla-*.whl"))


def _tcp_open(host: str, port: int, timeout: float) -> tuple[bool, str | None]:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True, None
    except Exception as exc:
        return False, _exception_text(exc)


def _empty_server(host: str, port: int, reason: str) -> dict:
    return {
        "host": host,
        "port": port,
        "connected": False,
        "server_version": None,
        "client_version": None,
        "available_maps": None,
        "synchronous_mode": None,
        "reason": reason,
    }


def _read_sync_settings(settings) -> dict:
    found = {}
    for name in SYNC_SETTING_FIELDS:
        if not hasattr(settings, name):
            continue
        try:
            found[name] = _json_safe(getattr(settings, name))
        except Exception as exc:
            found[name] = None
            found[f"{name}_error"] = _exception_text(exc)
    return found


def _probe_carla_server(host: str, port: int, timeout: float, package_reason: str | None) -> dict:
    tcp_open, tcp_error = _tcp_open(host, port, min(timeout, 2.0))
    tcp_note = (
        f"TCP {host}:{port} 可连接"
        if tcp_open
        else f"TCP {host}:{port} 不可连接 ({tcp_error})"
    )

    try:
        import carla
    except Exception as exc:
        reason = f"CARLA Python 包不可导入 ({_exception_text(exc)})，无法查询服务端。{tcp_note}"
        if package_reason and package_reason not in reason:
            reason = f"{package_reason}。{reason}"
        return _empty_server(host, port, reason)

    client = carla.Client(host, port)
    client.set_timeout(timeout)
    server = _empty_server(host, port, None)
    errors = []

    try:
        server["server_version"] = str(client.get_server_version())
        server["connected"] = True
    except Exception as exc:
        server["reason"] = (
            f"无法连接 CARLA 服务端 {host}:{port} ({_exception_text(exc)})。{tcp_note}"
        )
        return server

    try:
        server["client_version"] = str(client.get_client_version())
    except Exception as exc:
        errors.append(f"client_version: {_exception_text(exc)}")

    try:
        server["available_maps"] = [str(item) for item in client.get_available_maps()]
    except Exception as exc:
        errors.append(f"available_maps: {_exception_text(exc)}")

    try:
        world = client.get_world()
        server["synchronous_mode"] = _read_sync_settings(world.get_settings())
    except Exception as exc:
        errors.append(f"synchronous_mode: {_exception_text(exc)}")

    server["reason"] = "；".join(errors) if errors else None
    return server


def _probe_gpu() -> tuple[list | None, str | None]:
    executable = shutil.which("nvidia-smi")
    if executable is None:
        return None, None

    try:
        completed = subprocess.run(
            [
                executable,
                "--query-gpu=name,memory.total",
                "--format=csv,noheader,nounits",
            ],
            capture_output=True,
            text=True,
            timeout=8,
            check=False,
        )
    except Exception as exc:
        return None, f"nvidia-smi 执行失败 ({_exception_text(exc)})"

    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout or "").strip()
        return None, f"nvidia-smi 退出码 {completed.returncode}: {detail}"

    devices = []
    for line in (completed.stdout or "").splitlines():
        text = line.strip()
        if not text:
            continue
        name, separator, memory = text.rpartition(",")
        if not separator:
            devices.append({"name": text, "memory_total_mib": None})
            continue
        memory_text = memory.strip()
        try:
            memory_mib = int(float(memory_text))
        except ValueError:
            memory_mib = None
        devices.append({"name": name.strip(), "memory_total_mib": memory_mib})

    if not devices:
        return None, "nvidia-smi 没有返回显卡信息"
    return devices, None


def _probe_ffmpeg() -> dict:
    executable = shutil.which("ffmpeg")
    if executable is None:
        return {
            "available": False,
            "version": None,
            "path": None,
            "reason": "未在 PATH 中找到 ffmpeg",
        }
    version = None
    reason = None
    try:
        completed = subprocess.run(
            [executable, "-version"],
            capture_output=True,
            text=True,
            timeout=8,
            check=False,
        )
    except Exception as exc:
        reason = _exception_text(exc)
    else:
        first_line = (completed.stdout or completed.stderr or "").splitlines()
        if first_line:
            parts = first_line[0].split()
            # Typical first line: "ffmpeg version N-... Copyright ..."
            if len(parts) >= 3 and parts[1] == "version":
                version = parts[2]
            else:
                version = first_line[0].strip()
        if completed.returncode != 0 and version is None:
            reason = f"ffmpeg 退出码 {completed.returncode}"
    return {
        "available": reason is None,
        "version": version,
        "path": executable,
        "reason": reason,
    }


def _probe_viz() -> dict:
    return {
        "matplotlib": _probe_import(("matplotlib",), "matplotlib"),
        "pandas": _probe_import(("pandas",), "pandas"),
        "imageio": _probe_import(("imageio",), "imageio"),
        "opencv-python": _probe_import(
            ("opencv-python", "opencv-python-headless", "opencv-contrib-python"),
            "cv2",
        ),
        "ffmpeg": _probe_ffmpeg(),
    }


def build_report(host: str, port: int, timeout: float) -> dict:
    numpy_info = _probe_import(("numpy",), "numpy")
    pyyaml_info = _probe_import(("PyYAML", "pyyaml"), "yaml")
    carla_info = _probe_import(("carla",), "carla")
    carla_python = {
        "version": carla_info["version"],
        "importable": carla_info["available"],
        "distribution": carla_info["distribution"],
        "nearby_wheels": _nearby_carla_wheels(),
        "reason": None if carla_info["available"] else carla_info["reason"],
    }

    gpu, gpu_reason = _probe_gpu()
    return {
        "os": _probe_os(),
        "python": _probe_python(),
        "numpy": {
            "version": numpy_info["version"],
            "reason": None if numpy_info["available"] else numpy_info["reason"],
        },
        "pyyaml": {
            "version": pyyaml_info["version"],
            "reason": None if pyyaml_info["available"] else pyyaml_info["reason"],
        },
        "carla_python": carla_python,
        "carla_server": _probe_carla_server(
            host,
            port,
            timeout,
            None if carla_info["available"] else carla_info["reason"],
        ),
        "gpu": gpu,
        "gpu_reason": gpu_reason,
        "viz": _probe_viz(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="探测 CARLA AEB/AES 项目运行环境，只读，不安装依赖。")
    parser.add_argument("--host", default=HOST_DEFAULT, help="CARLA 服务端地址，默认 localhost")
    parser.add_argument("--port", type=int, default=PORT_DEFAULT, help="CARLA 服务端端口，默认 2000")
    parser.add_argument("--timeout", type=float, default=TIMEOUT_DEFAULT, help="连接超时（秒），默认 3")
    parser.add_argument(
        "--output",
        default=str(Path(__file__).resolve().parent / "env.json"),
        help="输出路径，默认脚本同目录下的 env.json",
    )
    args = parser.parse_args()

    report = build_report(args.host, args.port, args.timeout)
    output = Path(args.output)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        # Last-resort guard: a probe must not traceback out of the process
        # without leaving a readable reason. Normal failures are stored in JSON.
        fallback = Path(__file__).resolve().parent / "env.json"
        payload = {
            "os": _probe_os(),
            "python": _probe_python(),
            "numpy": {"version": None, "reason": "probe failed before package check"},
            "pyyaml": {"version": None, "reason": "probe failed before package check"},
            "carla_python": {
                "version": None,
                "importable": False,
                "distribution": None,
                "nearby_wheels": [],
                "reason": _exception_text(exc),
            },
            "carla_server": _empty_server(
                HOST_DEFAULT,
                PORT_DEFAULT,
                f"探测脚本异常退出 ({_exception_text(exc)})",
            ),
            "gpu": None,
            "gpu_reason": _exception_text(exc),
            "viz": {},
        }
        try:
            fallback.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        except Exception:
            print(_exception_text(exc), file=sys.stderr)
            raise SystemExit(1) from exc
        print(f"wrote {fallback}", file=sys.stderr)
        print(_exception_text(exc), file=sys.stderr)
        raise SystemExit(1) from exc
