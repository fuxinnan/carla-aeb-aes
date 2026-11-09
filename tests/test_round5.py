"""第 5 轮：固定 TTC、状态机和制动执行层。不启动 CARLA。"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from dataclasses import asdict, replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg", force=True)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from aeb.config import StateMachineConfig, load_aeb_config
from aeb.fixed_ttc import FixedTtcAeb
from aeb.state_machine import StateMachineAeb
from aeb import build_policy
from control.actuator import BrakeActuator
from experiment import build_manifest
from run_logger import RunLogger
from scenarios.closed_loop import make_summary, simulate_episode
from scenarios.geometry import make_straight_road
from scenarios.motion import rollout
from scenarios.spec import load_spec
from schemas import ControlRequest, EgoState, EpisodeSummary, ThreatResult, load_dataclass
from viz.aeb_diag import plot_aeb_diagnostic, plot_strategy_compare, write_state_diagram
from viz.replay_bev import export_bev_replay

PED_YAML = ROOT / "configs" / "scenarios" / "pedestrian_crossing_collision.yaml"
LEAD_YAML = ROOT / "configs" / "scenarios" / "lead_vehicle_brake_collision.yaml"
STRATEGIES = ("no_intervention", "fixed_ttc", "state_machine")


class Round5Test(unittest.TestCase):
    def test_fixed_ttc_has_no_memory(self) -> None:
        config = load_aeb_config().fixed_ttc
        policy = FixedTtcAeb(config)
        fresh = FixedTtcAeb(config)
        first = policy.decide(_ego(10.0), [], _threat(1.0))
        self.assertEqual(policy.aeb_state, "FULL")
        self.assertEqual(policy.last_transition.reason, "ttc_below_threshold")
        self.assertEqual(first.target_decel_mps2, config.target_decel_mps2)
        released = policy.decide(_ego(10.0), [], _threat(config.ttc_brake_s))
        self.assertEqual(policy.aeb_state, "IDLE")
        self.assertEqual(released.target_decel_mps2, 0.0)
        self.assertEqual(policy.last_transition.reason, "ttc_above_threshold")
        again = policy.decide(_ego(10.0), [], _threat(1.0))
        expected = fresh.decide(_ego(10.0), [], _threat(1.0))
        self.assertEqual(again, expected)
        self.assertEqual(policy.decide(_ego(10.0), [], _threat(None)).target_decel_mps2, 0.0)
        self.assertEqual(policy.last_transition.reason, "ttc_unavailable")

    def test_state_machine_confirms_holds_and_exits_with_reasons(self) -> None:
        base = _machine_config(min_hold_frames=10)
        policy = StateMachineAeb(base)
        reasons = []
        states = []
        for _ in range(3):
            request = policy.decide(_ego(10.0), [], _threat(1.0))
            states.append(policy.aeb_state)
            if policy.last_transition is not None:
                reasons.append(policy.last_transition.reason)
        self.assertEqual(states, ["WARN", "WARN", "PARTIAL"])
        self.assertEqual(reasons, ["ttc_below_warn", "ttc_below_partial"])
        self.assertEqual(request.target_decel_mps2, base.partial_decel_mps2)
        for _ in range(4):
            policy.decide(_ego(10.0), [], _threat(1.8))
            self.assertIsNone(policy.last_transition)
            self.assertEqual(policy.aeb_state, "PARTIAL")
        policy.decide(_ego(0.0), [], _threat(1.0))
        self.assertEqual(policy.aeb_state, "EXIT")
        self.assertEqual(policy.last_transition.reason, "stopped")

        lost = StateMachineAeb(replace(base, min_hold_frames=1))
        for _ in range(3):
            lost.decide(_ego(10.0), [], _threat(1.0))
        self.assertEqual(lost.aeb_state, "PARTIAL")
        for _ in range(3):
            lost.decide(_ego(10.0), [], _threat(None))
            self.assertEqual(lost.aeb_state, "PARTIAL")
        lost.decide(_ego(10.0), [], _threat(None))
        self.assertEqual(lost.aeb_state, "EXIT")
        self.assertEqual(lost.last_transition.reason, "target_lost")
        self.assertTrue(lost.last_transition.reason)

    def test_actuator_slew_makes_request_and_actual_differ(self) -> None:
        actuator = BrakeActuator(load_aeb_config().actuator, dt_s=0.05)
        actuator.config = replace(actuator.config, slew_decel_mps3=20.0)
        asked, actual = actuator.command(ControlRequest(0.0, 0.0, 0.0, None, target_decel_mps2=6.0))
        self.assertAlmostEqual(actual.applied_decel_mps2, 1.0)
        self.assertAlmostEqual(asked.target_decel_mps2, 6.0)
        self.assertAlmostEqual(asked.brake, 0.75)
        self.assertAlmostEqual(actual.brake, 0.125)
        self.assertNotAlmostEqual(asked.brake, actual.brake)

    def test_config_rejects_crossed_thresholds(self) -> None:
        source = (ROOT / "configs" / "aeb" / "round5.yaml").read_text(encoding="utf-8")
        broken = source.replace("ttc_warn_off_s: 2.8", "ttc_warn_off_s: 2.4")
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "bad.yaml"
            path.write_text(broken, encoding="utf-8")
            with self.assertRaises(ValueError):
                load_aeb_config(path)

    def test_old_control_json_still_loads(self) -> None:
        request = load_dataclass(
            ControlRequest,
            {"throttle": 0.0, "brake": 0.2, "steer": 0.0, "desired_speed_mps": 8.0},
        )
        self.assertIsNone(request.target_decel_mps2)
        summary = EpisodeSummary(
            run_id="old",
            scene_id="scene",
            strategy_name="no_intervention",
            outcome="collision",
            collided=True,
            min_distance_m=0.0,
            min_ttc_s=None,
            first_trigger_time_s=None,
            aeb_first_trigger_time_s=None,
            aes_first_trigger_time_s=None,
            peak_decel_mps2=0.0,
            speed_drop_mps=None,
            initial_speed_mps=10.0,
            final_speed_mps=10.0,
            duration_s=1.0,
            frame_count=2,
            invalid_frame_count=0,
        )
        plain = asdict(summary)
        del plain["brake_delay_frames"]
        loaded = load_dataclass(EpisodeSummary, plain)
        self.assertIsNone(loaded.brake_delay_frames)

    def test_closed_loop_matches_rollout_and_aeb_logs_are_complete(self) -> None:
        road = make_straight_road(length_m=220.0, step_m=1.0, yaw_rad=0.0)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for spec_path in (PED_YAML, LEAD_YAML):
                spec = load_spec(spec_path)
                plan = rollout(spec, road)
                runs = {}
                for name in STRATEGIES:
                    run_dir = root / f"{spec.scene_id}_{name}"
                    episode, summary = _write_run(spec, road, name, run_dir)
                    runs[name] = run_dir
                    self.assertGreater(summary.frame_count, 2)
                    self.assertIn(episode.end_reason, {"collision", "stop", "timeout"})
                    _assert_events_explain_states(self, run_dir)
                    if name == "no_intervention":
                        self.assertEqual(episode.trigger_frame, plan.trigger_frame)
                        self.assertEqual(episode.collision_frame, plan.collision_frame)
                        self.assertEqual(len(episode.steps), len(plan.steps))
                        for step, sample in zip(episode.steps, plan.steps):
                            self.assertAlmostEqual(step.ego.s_m, sample.ego.s_m, places=6)
                            self.assertAlmostEqual(step.target.x_m, sample.target.x_m, places=6)
                        self.assertIsNone(summary.brake_delay_frames)
                    else:
                        requested = [step.record.control_request.target_decel_mps2 for step in episode.steps]
                        self.assertTrue(any(value >= 0.5 for value in requested))
                        self.assertEqual(summary.brake_delay_frames, 1)
                        self.assertIsNotNone(summary.aeb_first_trigger_time_s)
                    diagnostic = plot_aeb_diagnostic(run_dir)
                    mermaid, state_png = write_state_diagram(run_dir)
                    self.assertTrue(diagnostic.read_bytes().startswith(b"\x89PNG"))
                    self.assertGreater(diagnostic.stat().st_size, 1000)
                    self.assertIn("stateDiagram-v2", mermaid.read_text(encoding="utf-8"))
                    self.assertTrue(state_png.read_bytes().startswith(b"\x89PNG"))
                compare = plot_strategy_compare([runs[name] for name in STRATEGIES], root / f"{spec.scene_id}_compare.png")
                self.assertGreater(compare.stat().st_size, 1000)
            replay = export_bev_replay(root / f"{load_spec(PED_YAML).scene_id}_fixed_ttc")
            self.assertGreater(replay.stat().st_size, 500)

    def test_packages_do_not_import_carla(self) -> None:
        for relative in ("aeb", "control"):
            for path in (ROOT / relative).rglob("*.py"):
                self.assertNotIn("import carla", path.read_text(encoding="utf-8"), path)
        for relative in ("scenarios/closed_loop.py", "viz/aeb_diag.py"):
            self.assertNotIn("import carla", (ROOT / relative).read_text(encoding="utf-8"))


def _write_run(spec, road, name: str, run_dir: Path):
    config = load_aeb_config()
    policy = build_policy(name, config, spec.ego_speed_mps)
    actuator = BrakeActuator(config.actuator, spec.fixed_delta_seconds)
    episode = simulate_episode(spec, road, policy, actuator, run_dir.name)
    manifest = build_manifest(
        run_id=run_dir.name,
        scene_id=spec.scene_id,
        strategy_name=policy.name,
        strategy_version=policy.version,
        strategy_config_path=config.path,
        carla_version=None,
        map_name=spec.map_name,
        fixed_delta_seconds=spec.fixed_delta_seconds,
        synchronous_mode=True,
        physics_substep={"substepping": False, "max_substep_delta_time_s": None, "max_substeps": None},
        python_seed=spec.seed,
        numpy_seed=spec.seed,
        traffic_manager_seed=spec.seed,
        weather={"cloudiness": 0.0},
    )
    manifest["scenario"] = spec.key_fields()
    outcome = "collision" if episode.collided else "avoided"
    with RunLogger(run_dir, manifest) as logger:
        for step in episode.steps:
            logger.log_frame(step.record)
            for track in step.tracks:
                logger.log_target(step.frame, step.sim_time_s, track)
            if step.transition is not None:
                logger.log_event(
                    "aeb_state",
                    step.sim_time_s,
                    {
                        "frame": step.frame,
                        "from": step.transition.previous,
                        "to": step.transition.current,
                        "reason": step.transition.reason,
                        "ttc_s": step.record.threat.ttc_s,
                    },
                )
            if step.trigger_edge:
                logger.log_event(
                    "target_trigger",
                    step.sim_time_s,
                    {"frame": step.frame, "distance_m": step.distance_m},
                )
            if step.overlapping:
                logger.log_event(
                    "collision",
                    step.sim_time_s,
                    {"frame": step.frame, "detector": "oriented_box"},
                )
        summary = make_summary(
            spec,
            run_dir.name,
            policy.name,
            [step.record for step in episode.steps],
            [step.gap_m for step in episode.steps],
            episode.collided,
            outcome,
        )
        logger.write_summary(summary)
        logger.log_event("scenario_end", episode.steps[-1].sim_time_s, {"outcome": outcome})
    return episode, summary


def _assert_events_explain_states(test: unittest.TestCase, run_dir: Path) -> None:
    frames = [json.loads(line) for line in (run_dir / "frames.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
    events = [
        json.loads(line)
        for line in (run_dir / "events.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip() and json.loads(line).get("event_type") == "aeb_state"
    ]
    previous = "IDLE"
    index = 0
    for frame in frames:
        state = frame["aeb_state"]
        if state == previous:
            continue
        test.assertLess(index, len(events), frame["frame"])
        detail = events[index]["detail"]
        test.assertEqual(detail["from"], previous)
        test.assertEqual(detail["to"], state)
        test.assertTrue(detail["reason"])
        index += 1
        previous = state
    test.assertEqual(index, len(events))


def _machine_config(min_hold_frames: int) -> StateMachineConfig:
    config = load_aeb_config().state_machine
    return replace(config, min_hold_frames=min_hold_frames, lost_hold_frames=2, clear_frames=2)


def _ego(speed: float) -> EgoState:
    return EgoState(
        x_m=0.0,
        y_m=0.0,
        z_m=0.0,
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=0.0,
        v_long_mps=speed,
        v_lat_mps=0.0,
        v_up_mps=0.0,
        speed_mps=speed,
        yaw_rate_radps=0.0,
        a_long_mps2=0.0,
        a_lat_mps2=0.0,
        steer_road_rad=None,
    )


def _threat(ttc: float | None) -> ThreatResult:
    if ttc is None:
        return ThreatResult(None, None, "no_track", None, None, None, None)
    return ThreatResult(ttc, 0.0, None, "tgt", True, 1.0, 1.0)

