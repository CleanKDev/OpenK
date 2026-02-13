"""Hardware-free smoke checks for OpenK scripts.

These tests stub out the hardware-facing pieces so we can exercise the
script entrypoints without connecting to physical motors or cameras.
"""

from __future__ import annotations

import pathlib
import sys

from lerobot.teleoperators import Teleoperator, TeleoperatorConfig


# Add project root and sibling lerobot repo to sys.path for imports.
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "lerobot"))


class _FakeRobot:
    """Minimal robot stub with the interface used by teleop/record."""

    name = "openk_stub"
    robot_type = "openk_stub"
    action_features = {"joint": float}
    observation_features = {"joint.pos": float}

    def __init__(self, *args, **kwargs):
        self.connected = False
        self.sent_actions: list[dict] = []

    @property
    def is_connected(self):
        return self.connected

    def connect(self, calibrate: bool = True):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def get_observation(self):
        return {"joint.pos": 0.0}

    def send_action(self, action):
        self.sent_actions.append(action)
        return action


class _FakeTeleop(Teleoperator):
    """Minimal Teleoperator subclass to satisfy isinstance checks."""

    config_class = TeleoperatorConfig
    name = "fake_teleop"

    def __init__(self):
        # Skip parent init to avoid filesystem writes
        self.connected = False
        self.val = 0.0

    @property
    def action_features(self):
        return {"joint": float}

    @property
    def feedback_features(self):
        return {}

    @property
    def is_connected(self):
        return self.connected

    def connect(self, calibrate: bool = True):
        self.connected = True

    @property
    def is_calibrated(self):
        return True

    def calibrate(self):
        return None

    def configure(self):
        return None

    def get_action(self):
        # Alternate sign to ensure actions change over time
        self.val = -self.val if self.val else 0.1
        return {"joint": self.val}

    def send_feedback(self, feedback: dict):
        return feedback

    def disconnect(self):
        self.connected = False


def test_teleop_loop_smoke(monkeypatch):
    from scripts import openk_teleoperate as teleop_script
    from lerobot.processor import make_default_processors

    fake_robot = _FakeRobot()
    fake_teleop = _FakeTeleop()
    tp, rp, op = make_default_processors()

    # Speed up test by making busy_wait a no-op
    monkeypatch.setattr(teleop_script, "busy_wait", lambda *_args, **_kwargs: None)

    teleop_script.teleop_loop(
        teleop=fake_teleop,
        robot=fake_robot,
        fps=10,
        teleop_action_processor=tp,
        robot_action_processor=rp,
        robot_observation_processor=op,
        display_data=False,
        duration=0.05,  # small duration to exit quickly
    )

    assert fake_robot.sent_actions, "teleop loop should have sent at least one action"


def test_record_loop_smoke(monkeypatch):
    sys.path.insert(0, str(ROOT))  # ensure local scripts module is found
    from scripts import openk_record as record_script
    from lerobot.processor import make_default_processors

    fake_robot = _FakeRobot()
    fake_teleop = _FakeTeleop()
    tp, rp, op = make_default_processors()

    events = {"stop_recording": False, "exit_early": False, "rerecord_episode": False}

    monkeypatch.setattr(record_script, "busy_wait", lambda *_args, **_kwargs: None)

    record_script.record_loop(
        robot=fake_robot,
        events=events,
        fps=10,
        teleop_action_processor=tp,
        robot_action_processor=rp,
        robot_observation_processor=op,
        dataset=None,  # avoid disk I/O
        teleop=fake_teleop,
        policy=None,
        preprocessor=None,
        postprocessor=None,
        control_time_s=0.05,
        single_task=None,
        display_data=False,
    )

    assert fake_robot.sent_actions, "record loop should have sent at least one action"


def test_calibrate_entrypoint_monkeypatched(monkeypatch):
    """Ensure calibrate() runs when the device is mocked."""
    from scripts import lerobot_calibrate as calibrate_script
    from lerobot.robots import RobotConfig

    # Replace make_robot_from_config / make_teleoperator_from_config to return fake devices
    monkeypatch.setattr(calibrate_script, "make_robot_from_config", lambda *_args, **_kwargs: _FakeRobot())
    monkeypatch.setattr(
        calibrate_script,
        "make_teleoperator_from_config",
        lambda *_args, **_kwargs: _FakeTeleop(),
    )

    # Keep track of calls to ensure flow executed
    calls = {"connect": 0, "calibrate": 0, "disconnect": 0}

    class _Device(_FakeRobot):
        def connect(self, calibrate: bool = True):
            calls["connect"] += 1

        def calibrate(self):
            calls["calibrate"] += 1

        def disconnect(self):
            calls["disconnect"] += 1

    # Return our tracked device regardless of config type
    monkeypatch.setattr(calibrate_script, "make_robot_from_config", lambda *_args, **_kwargs: _Device())
    monkeypatch.setattr(calibrate_script, "make_teleoperator_from_config", lambda *_args, **_kwargs: _Device())

    class _DummyRobotConfig(RobotConfig):
        """Lightweight RobotConfig subclass for calibration smoke test."""

        # Skip Camera validation in base __post_init__
        def __post_init__(self):
            return None

    cfg = calibrate_script.CalibrateConfig(robot=_DummyRobotConfig())
    calibrate_script.calibrate(cfg)

    assert calls == {"connect": 1, "calibrate": 1, "disconnect": 1}
