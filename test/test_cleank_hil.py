"""Hardware-in-the-loop checks for OpenK (requires real devices).

These tests only run when ``OPENK_HIL=1`` is set. Provide device paths via:
  - OPENK_FOLLOWER_PORT (e.g., /dev/ttyUSB0)
  - OPENK_LEADER_PORT (e.g., /dev/ttyUSB1)

They perform minimal, low-risk operations: connect, read one observation/action,
and disconnect. Camera usage is disabled to avoid needing a RealSense endpoint.
"""

from __future__ import annotations

import os
import pathlib
import sys
import pytest

# Add project root and sibling lerobot repo to sys.path for imports.
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "lerobot"))

from scripts.openk_teleoperate import _register_openk
from lerobot.robots import RobotConfig, make_robot_from_config
from lerobot.teleoperators import TeleoperatorConfig, make_teleoperator_from_config

pytestmark = pytest.mark.skipif(
    os.getenv("OPENK_HIL") != "1",
    reason="Set OPENK_HIL=1 to run hardware-in-the-loop tests",
)


def _require_env(name: str) -> str:
    val = os.getenv(name)
    if not val:
        pytest.skip(f"{name} not set")
    return val


def _make_follower_config(port: str):
    # Ensure OpenK types are registered
    _register_openk()
    cfg_cls = RobotConfig.get_choice_class("openk-1-alpha_follower")
    # Disable cameras to keep the test lightweight/safe if a RealSense is not attached
    return cfg_cls(port=port, cameras={})


def _make_leader_config(port: str):
    _register_openk()
    cfg_cls = TeleoperatorConfig.get_choice_class("openk_leader")
    return cfg_cls(port=port)


def test_follower_connect_and_observe():
    port = _require_env("OPENK_FOLLOWER_PORT")
    follower_cfg = _make_follower_config(port)
    robot = make_robot_from_config(follower_cfg)

    robot.connect(calibrate=False)
    try:
        obs = robot.get_observation()
        assert isinstance(obs, dict)
        assert obs, "expected at least one observation value"
    finally:
        robot.disconnect()


def test_leader_connect_and_get_action():
    port = _require_env("OPENK_LEADER_PORT")
    leader_cfg = _make_leader_config(port)
    teleop = make_teleoperator_from_config(leader_cfg)

    teleop.connect(calibrate=False)
    try:
        action = teleop.get_action()
        assert isinstance(action, dict)
        assert action, "expected at least one action value"
    finally:
        teleop.disconnect()
