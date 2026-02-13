"""Smoke tests to ensure OpenK follower/leader types register without touching hardware."""

from __future__ import annotations

import pathlib
import sys


# Make sure local sources (and the sibling lerobot repo) are on sys.path.
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "lerobot"))


def test_openk_types_registered():
    from scripts.openk_teleoperate import _register_openk
    from lerobot.robots import RobotConfig
    from lerobot.teleoperators import TeleoperatorConfig

    _register_openk()

    robot_types = RobotConfig.get_known_choices().keys()
    teleop_types = TeleoperatorConfig.get_known_choices().keys()

    assert "openk-1-alpha_follower" in robot_types
    assert "openk_leader" in teleop_types
