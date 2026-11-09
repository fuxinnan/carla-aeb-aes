"""CARLA 同步会话。退出时销毁自车和传感器，并把服务端从同步模式恢复回来。"""

from __future__ import annotations

import math
import queue
import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if not any(Path(entry).resolve() == _PROJECT_ROOT for entry in sys.path if entry):
    sys.path.insert(0, str(_PROJECT_ROOT))

import carla

from schemas import EgoState

ROLE_NAME = "ego"
VEHICLE_BLUEPRINT = "vehicle.lincoln.mkz_2020"
SPAWN_INDEX = 0
TM_PORT = 8000


class CarlaSession:
    """进入时打开同步和固定时步，退出时一定关回去。

    0.9.16 的 Traffic Manager 没有读取同步开关的接口。地图重新加载后它的默认状态是异步，
    这里把这个默认值当作备份；退出时写回异步。世界设置用 get_settings() 的原对象备份。
    """

    def __init__(
        self,
        *,
        host: str = "localhost",
        port: int = 2000,
        map_name: str,
        fixed_delta_seconds: float,
        render: bool,
        tm_port: int = TM_PORT,
        role_name: str = ROLE_NAME,
        vehicle_blueprint: str = VEHICLE_BLUEPRINT,
        spawn_index: int = SPAWN_INDEX,
        traffic_manager_seed: int = 0,
        debug_draw: bool = False,
        spawn_ego: bool = True,
    ) -> None:
        if fixed_delta_seconds <= 0.0:
            raise ValueError("固定时步必须大于 0")
        self.host = host
        self.port = port
        self.map_name = map_name
        self.fixed_delta_seconds = float(fixed_delta_seconds)
        self.render = bool(render)
        self.tm_port = int(tm_port)
        self.role_name = role_name
        self.vehicle_blueprint = vehicle_blueprint
        self.spawn_index = int(spawn_index)
        self.traffic_manager_seed = int(traffic_manager_seed)
        self.debug_draw = bool(debug_draw)
        self.spawn_ego = bool(spawn_ego)

        self.client: carla.Client | None = None
        self.world: carla.World | None = None
        self.ego: carla.Actor | None = None
        self.tm = None
        self.original_settings = None
        self.server_version: str | None = None
        self.spawn_pose: dict | None = None
        self._sensors: list = []
        self._owned: list = []
        self._imu_queue: queue.Queue = queue.Queue()
        self._collision_queue: queue.Queue = queue.Queue()
        self._tm_sync_backup = False
        self._sync_applied = False
        self._shutdown_done = False

    def __enter__(self) -> "CarlaSession":
        try:
            self._setup()
        except BaseException:
            self.shutdown()
            raise
        return self

    def __exit__(self, exc_type, exc, traceback) -> bool:
        self.shutdown()
        return False

    def read_snapshot(self):
        """读取当前帧快照。仿真时间用这里的 elapsed_seconds，不用墙钟。"""
        self._require_world()
        return self.world.get_snapshot()

    def reload_episode(self) -> None:
        """清掉上一轮的车辆、行人和传感器，并重新打开同步。

        0.9.16 里连续 load_world / reload_world 会让客户端在 Simulator.h 断言
        `_episode != nullptr` 后直接退出。地图本身没变，所以重复实验只清 actor。
        """
        self._destroy_dynamic_actors()
        self._sensors.clear()
        self._owned.clear()
        self.ego = None
        self._enable_sync(self.world)

    def tick(self) -> int:
        """只推进一个固定时步，不改写已经发给车辆的控制量。"""
        self._require_world()
        return int(self.world.tick())

    def track_actor(self, actor) -> None:
        """退出时和自车一起销毁。场景脚本生成的目标走这里。"""
        self._owned.append(actor)

    def advance(self) -> int:
        """控制量写零，可选地画一拍调试标记，然后推进一个固定时步。"""
        self._require_world()
        if self.ego is None:
            raise RuntimeError("自车还没有生成")
        self.ego.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                steer=0.0,
                brake=0.0,
                hand_brake=False,
                reverse=False,
                manual_gear_shift=False,
            )
        )
        if self.debug_draw:
            self.draw_debug(self.fixed_delta_seconds)
        return int(self.world.tick())

    def collect(self, tick_frame: int):
        """收集这一拍的车辆状态、IMU 和碰撞。tick() 返回后传感器数据已经进队列。"""
        self._require_world()
        snapshot = self.world.get_snapshot()
        imu_data, imu_ok = self._take_imu(tick_frame)
        collisions = self._take_collisions(tick_frame)
        ego_state, used_imu = build_ego_state(self.ego, imu_data if imu_ok else None)
        return {
            "snapshot": snapshot,
            "elapsed_seconds": float(snapshot.timestamp.elapsed_seconds),
            "carla_frame": int(snapshot.timestamp.frame),
            "ego": ego_state,
            "imu_ok": used_imu,
            "collisions": collisions,
        }

    def draw_debug(self, life_time_s: float) -> None:
        """包围盒和速度文字只保留这一拍。life_time 用固定时步，单位秒。"""
        if self.ego is None or self.world is None:
            return
        transform = self.ego.get_transform()
        box = self.ego.bounding_box
        center = transform.transform(box.location)
        self.world.debug.draw_box(
            carla.BoundingBox(center, box.extent),
            transform.rotation,
            thickness=0.12,
            color=carla.Color(0, 220, 0),
            life_time=float(life_time_s),
        )
        velocity = self.ego.get_velocity()
        speed = math.hypot(velocity.x, velocity.y)
        self.world.debug.draw_string(
            transform.location + carla.Location(z=2.0),
            f"{speed:.2f} m/s",
            False,
            carla.Color(255, 255, 255),
            float(life_time_s),
        )

    def weather_dict(self) -> dict:
        self._require_world()
        weather = self.world.get_weather()
        fields = (
            "cloudiness",
            "precipitation",
            "precipitation_deposits",
            "wind_intensity",
            "sun_azimuth_angle",
            "sun_altitude_angle",
            "fog_density",
            "fog_distance",
            "fog_falloff",
            "wetness",
        )
        return {name: float(getattr(weather, name)) for name in fields if hasattr(weather, name)}

    def physics_substep(self) -> dict:
        self._require_world()
        settings = self.world.get_settings()
        return {
            "substepping": bool(getattr(settings, "substepping", False)),
            "max_substep_delta_time_s": _optional_float(getattr(settings, "max_substep_delta_time", None)),
            "max_substeps": _optional_int(getattr(settings, "max_substeps", None)),
        }

    def shutdown(self) -> None:
        """销毁传感器和自车，恢复交通管理器和世界设置，关闭同步。"""
        if self._shutdown_done:
            return
        self._shutdown_done = True
        for sensor in self._sensors:
            try:
                sensor.stop()
            except Exception:
                pass
        for actor in list(self._sensors) + list(self._owned) + ([self.ego] if self.ego is not None else []):
            try:
                actor.destroy()
            except Exception:
                pass
        self._sensors.clear()
        self._owned.clear()
        self.ego = None
        if self.world is not None and self._sync_applied:
            try:
                self.world.tick()
            except Exception:
                pass
        if self.tm is not None:
            try:
                self.tm.set_synchronous_mode(self._tm_sync_backup)
            except Exception:
                pass
        if self.world is not None and self.original_settings is not None:
            try:
                self.world.apply_settings(self.original_settings)
            except Exception:
                pass
        self._sync_applied = False

    def _setup(self) -> None:
        client = carla.Client(self.host, self.port)
        client.set_timeout(10.0)
        self.client = client
        self._release_stuck_sync(client)
        client.set_timeout(120.0)
        self.server_version = str(client.get_server_version())
        resolved = self._resolve_map(client, self.map_name)
        world = _existing_world(client, resolved, self.map_name)
        if world is None:
            world = client.load_world(resolved)
        self.world = world
        self._destroy_dynamic_actors()
        # 备份的是加载地图之后、打开同步之前的设置。正常情况下同步是关的。
        self.original_settings = world.get_settings()
        self.tm = client.get_trafficmanager(self.tm_port)
        self._tm_sync_backup = False
        self.tm.set_synchronous_mode(False)
        self.tm.set_random_device_seed(self.traffic_manager_seed)
        self._enable_sync(world)

        if self.spawn_ego:
            blueprint = world.get_blueprint_library().find(self.vehicle_blueprint)
            if blueprint.has_attribute("role_name"):
                blueprint.set_attribute("role_name", self.role_name)
            spawn_points = world.get_map().get_spawn_points()
            if not spawn_points:
                raise RuntimeError(f"{self.map_name} 没有出生点")
            if self.spawn_index < 0 or self.spawn_index >= len(spawn_points):
                raise RuntimeError(f"出生点序号 {self.spawn_index} 超出范围 0..{len(spawn_points) - 1}")
            spawn = spawn_points[self.spawn_index]
            self.ego = world.try_spawn_actor(blueprint, spawn)
            if self.ego is None:
                raise RuntimeError(f"出生点 {self.spawn_index} 无法生成 {self.vehicle_blueprint}")
            self.spawn_pose = _pose_dict(self.ego.get_transform())
            self._spawn_sensors()
        client.set_timeout(20.0)

    def _enable_sync(self, world) -> None:
        self.tm = self.client.get_trafficmanager(self.tm_port)
        self.tm.set_random_device_seed(self.traffic_manager_seed)
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.fixed_delta_seconds
        settings.no_rendering_mode = not self.render
        world.apply_settings(settings)
        self.tm.set_synchronous_mode(True)
        self._sync_applied = True

    def _destroy_dynamic_actors(self) -> None:
        """清掉车辆、行人和传感器。地图重载后调用，避免上一轮的 actor 还占着路。"""
        if self.world is None or self.client is None:
            return
        destroy_ids = []
        for actor in self.world.get_actors():
            type_id = str(actor.type_id)
            if type_id.startswith(("vehicle.", "walker.", "sensor.", "controller.")):
                destroy_ids.append(int(actor.id))
        if not destroy_ids:
            return
        try:
            self.client.apply_batch([carla.command.DestroyActor(actor_id) for actor_id in destroy_ids])
        except Exception:
            for actor in self.world.get_actors():
                if int(actor.id) in destroy_ids:
                    try:
                        actor.destroy()
                    except Exception:
                        pass

    def _spawn_sensors(self) -> None:
        library = self.world.get_blueprint_library()
        identity = carla.Transform()
        collision = self.world.spawn_actor(
            library.find("sensor.other.collision"),
            identity,
            attach_to=self.ego,
        )
        imu = self.world.spawn_actor(
            library.find("sensor.other.imu"),
            identity,
            attach_to=self.ego,
        )
        collision.listen(self._collision_queue.put)
        imu.listen(self._imu_queue.put)
        self._sensors.extend((collision, imu))

    def _resolve_map(self, client, name: str) -> str:
        available = list(client.get_available_maps())
        if name in available:
            return name
        matches = [item for item in available if item.endswith("/" + name) or item.endswith(name)]
        if matches:
            return matches[0]
        raise RuntimeError(f"找不到地图 {name}。服务端地图：{available}")

    def _release_stuck_sync(self, client) -> None:
        """上次若停在同步模式，服务端会一直等 tick。先推进一步，再关同步，避免必须重启服务端。"""
        client.set_timeout(8.0)
        try:
            world = client.get_world()
            settings = world.get_settings()
        except Exception:
            return
        if not bool(settings.synchronous_mode):
            try:
                client.get_trafficmanager(self.tm_port).set_synchronous_mode(False)
            except Exception:
                pass
            return
        try:
            world.tick()
        except Exception:
            pass
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        try:
            world.apply_settings(settings)
        except Exception:
            try:
                world.tick()
                world.apply_settings(settings)
            except Exception:
                pass
        try:
            client.get_trafficmanager(self.tm_port).set_synchronous_mode(False)
        except Exception:
            pass

    def _take_imu(self, tick_frame: int):
        try:
            data = _take_sensor_frame(self._imu_queue, tick_frame, timeout_s=2.0)
        except queue.Empty:
            return None, False
        return data, True

    def _take_collisions(self, tick_frame: int) -> list:
        found = []
        while True:
            try:
                event = self._collision_queue.get_nowait()
            except queue.Empty:
                break
            if int(event.frame) <= tick_frame:
                found.append(event)
            else:
                self._collision_queue.put(event)
                break
        return found

    def _require_world(self) -> None:
        if self.world is None:
            raise RuntimeError("CARLA 世界还没有打开")


def build_ego_state(actor, imu_data) -> tuple[EgoState, bool]:
    """位置和速度来自车辆真值。加速度和横摆角速度优先用 IMU，缺测时退回车辆真值。"""
    transform = actor.get_transform()
    velocity = actor.get_velocity()
    forward = transform.get_forward_vector()
    right = transform.get_right_vector()
    up = transform.get_up_vector()
    used_imu = imu_data is not None
    if used_imu:
        accel_x = float(imu_data.accelerometer.x)
        accel_y = float(imu_data.accelerometer.y)
        yaw_rate = float(imu_data.gyroscope.z)
    else:
        acceleration = actor.get_acceleration()
        accel_x = _dot(acceleration, forward)
        accel_y = _dot(acceleration, right)
        angular = actor.get_angular_velocity()
        yaw_rate = math.radians(float(angular.z))
    return (
        EgoState(
            x_m=float(transform.location.x),
            y_m=float(transform.location.y),
            z_m=float(transform.location.z),
            roll_rad=math.radians(float(transform.rotation.roll)),
            pitch_rad=math.radians(float(transform.rotation.pitch)),
            yaw_rad=math.radians(float(transform.rotation.yaw)),
            v_long_mps=_dot(velocity, forward),
            v_lat_mps=_dot(velocity, right),
            v_up_mps=_dot(velocity, up),
            speed_mps=math.hypot(float(velocity.x), float(velocity.y)),
            yaw_rate_radps=yaw_rate,
            a_long_mps2=accel_x,
            a_lat_mps2=accel_y,
            steer_road_rad=None,
        ),
        used_imu,
    )


def _existing_world(client, resolved: str, short_name: str):
    """地图已经是目标地图、而且不在同步模式时，直接用它，避免再 load_world。"""
    try:
        world = client.get_world()
        current = str(world.get_map().name).replace("\\", "/")
        resolved_tail = str(resolved).replace("\\", "/").split("/")[-1]
        current_tail = current.split("/")[-1]
        same = current_tail in {resolved_tail, short_name} or current.endswith("/" + short_name)
        if same and not bool(world.get_settings().synchronous_mode):
            return world
    except Exception:
        return None
    return None


def server_synchronous_mode(host: str, port: int) -> bool:
    """再连一次，只读当前世界是不是还停在同步模式。"""
    client = carla.Client(host, port)
    client.set_timeout(8.0)
    return bool(client.get_world().get_settings().synchronous_mode)


def _take_sensor_frame(sensor_queue: queue.Queue, tick_frame: int, timeout_s: float):
    import time

    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise queue.Empty
        data = sensor_queue.get(timeout=remaining)
        if int(data.frame) == tick_frame:
            return data
        if int(data.frame) > tick_frame:
            sensor_queue.put(data)
            raise queue.Empty


def _pose_dict(transform) -> dict:
    return {
        "x_m": float(transform.location.x),
        "y_m": float(transform.location.y),
        "z_m": float(transform.location.z),
        "roll_rad": math.radians(float(transform.rotation.roll)),
        "pitch_rad": math.radians(float(transform.rotation.pitch)),
        "yaw_rad": math.radians(float(transform.rotation.yaw)),
    }


def _dot(vector, axis) -> float:
    return float(vector.x) * float(axis.x) + float(vector.y) * float(axis.y) + float(vector.z) * float(axis.z)


def _optional_float(value):
    if value is None:
        return None
    return float(value)


def _optional_int(value):
    if value is None:
        return None
    return int(value)
