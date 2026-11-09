"""把一幕运动学结果放进 CARLA 同步世界。

决策和积分仍由原来的闭环完成。这里负责把每一帧贴到地图直道上、移动镜头，
并用 world.tick() 推进服务端。没有服务端时，实验入口会在这里失败。
"""

from __future__ import annotations

import math

from scenarios.geometry import pose_on_road, wrap_pi
from sim.debug_draw import draw_replay_frame


MAP_NAME = "Town10HD_Opt"
HOST = "localhost"
PORT = 2000
DT_S = 0.05
ROAD_LENGTH_M = 120.0
ROAD_STEP_M = 2.0
EGO_BLUEPRINT = "vehicle.lincoln.mkz_2020"
BLUEPRINTS = {
    "vehicle": "vehicle.tesla.model3",
    "pedestrian": "walker.pedestrian.0001",
}


def world_pose(road, s_m: float, d_m: float, yaw_rad: float) -> tuple[float, float, float, float]:
    """合成直线上的弧长、向右横向偏移和相对航向，换成地图世界位姿。"""
    x_m, y_m, z_m, road_yaw = pose_on_road(road, float(s_m), float(d_m))
    return x_m, y_m, z_m, wrap_pi(road_yaw + float(yaw_rad))


def follow_spectator(world, x_m: float, y_m: float, z_m: float, yaw_rad: float, *, back_m: float = 12.0, height_m: float = 6.0, pitch_deg: float = -18.0) -> None:
    """把观察相机放到车后上方。要在 world.tick() 之前调用，这一拍画面才跟得上。"""
    import carla

    location = carla.Location(
        x=float(x_m) - math.cos(yaw_rad) * back_m,
        y=float(y_m) - math.sin(yaw_rad) * back_m,
        z=float(z_m) + height_m,
    )
    world.get_spectator().set_transform(
        carla.Transform(location, carla.Rotation(pitch=float(pitch_deg), yaw=math.degrees(yaw_rad), roll=0.0))
    )


class CarlaStage:
    """一场实验共用一个世界。换场景只清演员，不重新加载地图。"""

    def __init__(
        self,
        *,
        host: str = HOST,
        port: int = PORT,
        map_name: str = MAP_NAME,
        fixed_delta_seconds: float = DT_S,
        render: bool = True,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.map_name = map_name
        self.fixed_delta_seconds = float(fixed_delta_seconds)
        self.render = bool(render)
        self.session = None
        self.road = None
        self.server_version: str | None = None

    def __enter__(self) -> "CarlaStage":
        from scenarios.placement import find_straight_road
        from sim.carla_session import CarlaSession

        self.session = CarlaSession(
            host=self.host,
            port=self.port,
            map_name=self.map_name,
            fixed_delta_seconds=self.fixed_delta_seconds,
            render=self.render,
            spawn_ego=False,
        )
        try:
            self.session.__enter__()
            self.server_version = self.session.server_version
            self.road, _spawn_index = find_straight_road(self.session.world, ROAD_LENGTH_M, ROAD_STEP_M)
        except BaseException:
            import sys

            session = self.session
            self.session = None
            if session is not None:
                session.__exit__(*sys.exc_info())
            raise
        print(f"carla ready map={self.map_name} server={self.server_version}", flush=True)
        return self

    def __exit__(self, exc_type, exc, traceback) -> bool:
        from sim.carla_session import server_synchronous_mode

        session = self.session
        self.session = None
        if session is not None:
            session.__exit__(exc_type, exc, traceback)
        if exc_type is None and server_synchronous_mode(self.host, self.port):
            raise RuntimeError("退出后服务端仍停在同步模式")
        return False

    def playback_meta(self) -> dict:
        return {
            "map": self.map_name,
            "host": self.host,
            "port": self.port,
            "server_version": self.server_version,
            "synchronous_mode": True,
            "render": self.render,
        }

    def play(
        self,
        samples: list[dict],
        records: list | None = None,
        *,
        dt_s: float = DT_S,
        hold_s: float = 0.0,
        pace_s: float = 0.0,
        back_m: float = 12.0,
        height_m: float = 6.0,
        pitch_deg: float = -18.0,
    ) -> None:
        """samples 里的 x 是弧长，y 是向右的横向偏移。records 与帧对齐，用来给框上色。

        pace_s 是每一帧至少占用的墙钟秒数。实验批跑保持 0，观看时用它把画面放慢。
        """
        import time

        if self.session is None or self.road is None:
            raise RuntimeError("CARLA 世界还没有打开")
        if not samples:
            raise ValueError("没有可回放的帧")
        if records is not None and len(records) != len(samples):
            raise ValueError(f"帧数 {len(samples)} 和记录数 {len(records)} 不一致")
        if pace_s < 0.0:
            raise ValueError("pace_s 不能为负")
        self.session.reload_episode()
        actors = _spawn_cast(self.session, self.road, samples[0])
        previous: dict | None = None
        frames = list(samples)
        if hold_s > 0.0:
            extra = max(1, int(round(hold_s / dt_s)))
            frames.extend([samples[-1]] * extra)
        life_s = max(dt_s, pace_s)
        for index, sample in enumerate(frames):
            started = time.perf_counter()
            record = None if records is None or index >= len(records) else records[index]
            previous = _show_frame(
                self.session,
                self.road,
                actors,
                sample,
                record,
                previous,
                dt_s,
                life_s,
                back_m,
                height_m,
                pitch_deg,
            )
            if pace_s > 0.0:
                remaining = pace_s - (time.perf_counter() - started)
                if remaining > 0.0:
                    time.sleep(remaining)


def _spawn_cast(session, road, sample: dict) -> dict:
    actors = {}
    ego = sample["ego"]
    actors["ego"] = _spawn(session, road, EGO_BLUEPRINT, "ego", ego, 4.90, 2.12, float(ego.get("height_m", 1.51)))
    for body in sample["bodies"]:
        kind = _kind(body)
        actors[body["track_id"]] = _spawn(
            session,
            road,
            BLUEPRINTS[kind],
            str(body["track_id"]),
            body,
            float(body["length_m"]),
            float(body["width_m"]),
            float(body.get("height_m", 1.80 if kind == "pedestrian" else 1.51)),
        )
    return actors


def _show_frame(session, road, actors: dict, sample: dict, record, previous: dict | None, dt_s: float, life_s: float, back_m: float, height_m: float, pitch_deg: float) -> dict:
    poses = {"ego": world_pose(road, sample["ego"]["x_m"], sample["ego"]["y_m"], sample["ego"]["yaw_rad"])}
    for body in sample["bodies"]:
        poses[body["track_id"]] = world_pose(road, body["x_m"], body["y_m"], body["yaw_rad"])
    for key, pose in poses.items():
        vx, vy = (0.0, 0.0) if previous is None or key not in previous else ((pose[0] - previous[key][0]) / dt_s, (pose[1] - previous[key][1]) / dt_s)
        _place(actors[key]["actor"], pose, vx, vy, actors[key]["z_lift"])
    ego_pose = poses["ego"]
    follow_spectator(session.world, ego_pose[0], ego_pose[1], ego_pose[2], ego_pose[3], back_m=back_m, height_m=height_m, pitch_deg=pitch_deg)
    threat = None if record is None else record.threat
    state = "IDLE" if record is None else record.aeb_state
    notice = None
    if record is not None and record.trigger_reason:
        notice = f"AEB {record.aeb_state} {record.trigger_reason}"
    others = []
    for body in sample["bodies"]:
        cast = actors[body["track_id"]]
        others.append((cast["actor"], cast["length_m"], cast["width_m"], cast["height_m"], body["track_id"]))
    ego_cast = actors["ego"]
    draw_replay_frame(
        session.world,
        ego_cast["actor"],
        others,
        ego_length_m=ego_cast["length_m"],
        ego_width_m=ego_cast["width_m"],
        ego_height_m=ego_cast["height_m"],
        threat=threat,
        life_time_s=life_s,
        aeb_state=state,
        notice=notice,
    )
    session.tick()
    return poses


def _kind(body: dict) -> str:
    name = str(body.get("class_name", ""))
    if name in BLUEPRINTS:
        return name
    if "ped" in str(body.get("track_id", "")):
        return "pedestrian"
    return "vehicle"


def _spawn(session, road, blueprint_id: str, role: str, body: dict, length_m: float, width_m: float, height_m: float) -> dict:
    import carla

    world = session.world
    blueprint = world.get_blueprint_library().find(blueprint_id)
    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute("role_name", role)
    if blueprint.has_attribute("is_invincible"):
        blueprint.set_attribute("is_invincible", "true")
    pose = world_pose(road, body["x_m"], body["y_m"], body["yaw_rad"])
    actor = None
    used_lift = 0.0
    for lift in (0.0, 0.4, 1.0):
        transform = carla.Transform(
            carla.Location(x=pose[0], y=pose[1], z=pose[2] + lift),
            carla.Rotation(pitch=0.0, yaw=math.degrees(pose[3]), roll=0.0),
        )
        actor = world.try_spawn_actor(blueprint, transform)
        if actor is not None:
            used_lift = lift
            break
    if actor is None:
        raise RuntimeError(f"无法生成 {blueprint_id}，位置 ({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})")
    actor.set_simulate_physics(False)
    session.track_actor(actor)
    return {"actor": actor, "length_m": length_m, "width_m": width_m, "height_m": height_m, "z_lift": used_lift}


def _place(actor, pose: tuple[float, float, float, float], vx: float, vy: float, z_lift: float) -> None:
    import carla

    actor.set_transform(
        carla.Transform(
            carla.Location(x=pose[0], y=pose[1], z=pose[2] + z_lift),
            carla.Rotation(pitch=0.0, yaw=math.degrees(pose[3]), roll=0.0),
        )
    )
    actor.set_target_velocity(carla.Vector3D(x=float(vx), y=float(vy), z=0.0))


def main(argv: list[str] | None = None) -> int:
    """在 CARLA 窗口里播放一条第 9 轮场景。"""
    import argparse
    from dataclasses import replace

    from aeb import build_policy, load_aeb_config
    from control.actuator import BrakeActuator
    from round9.catalog import load_scene
    from round9.simulate import DT_S, simulate_scene

    parser = argparse.ArgumentParser(description="把一条场景放进 CARLA 同步世界里播放。")
    parser.add_argument("--scene", required=True, help="第 9 轮场景 YAML，只用来选模板")
    parser.add_argument("--strategy", default="aeb_aes", choices=["no_intervention", "fixed_ttc", "state_machine", "aeb_aes"])
    parser.add_argument("--aeb-config", default=None, help="AEB YAML，默认 configs/aeb/round5.yaml")
    parser.add_argument("--occlusion", action="store_true", help="打开遮挡。遮挡场景不传这个开关时行人始终可见")
    parser.add_argument("--hold", type=float, default=2.0, help="结束后在最后一帧停留的秒数")
    parser.add_argument("--ego-speed", type=float, default=5.0, help="观看用车速，米/秒。默认 5，约 18 km/h")
    parser.add_argument("--distance", type=float, default=36.0, help="观看用初始间距，米。默认 36")
    parser.add_argument("--timeout", type=float, default=16.0, help="这一幕最长仿真秒数")
    parser.add_argument("--pace", type=float, default=0.15, help="每一帧在屏幕上停留的秒数。0 表示按机器最快速度播")
    args = parser.parse_args(argv)
    if args.ego_speed <= 0.0 or args.distance <= 0.0 or args.timeout <= 0.0 or args.pace < 0.0:
        raise SystemExit("车速、间距、超时必须大于 0，pace 不能为负")
    loaded = load_scene(args.scene)
    scene = replace(loaded, ego_speed_mps=float(args.ego_speed), distance_m=float(args.distance))
    config = load_aeb_config(args.aeb_config)
    policy = build_policy(args.strategy, config, scene.ego_speed_mps)
    result = simulate_scene(
        scene,
        policy,
        BrakeActuator(config.actuator, DT_S),
        occlusion=args.occlusion,
        timeout_s=float(args.timeout),
    )
    print(
        f"view speed_mps={scene.ego_speed_mps:.1f} distance_m={scene.distance_m:.1f} "
        f"pace_s={args.pace:.2f} file_was={loaded.ego_speed_mps:.1f}m/s,{loaded.distance_m:.1f}m",
        flush=True,
    )
    with CarlaStage() as stage:
        print(f"play {loaded.scene_id} {args.strategy} frames={len(result.samples)}", flush=True)
        stage.play(
            result.samples,
            result.records,
            dt_s=DT_S,
            hold_s=args.hold,
            pace_s=float(args.pace),
            back_m=22.0,
            height_m=14.0,
            pitch_deg=-28.0,
        )
    print(f"outcome={result.summary.outcome} collided={int(result.collided)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
