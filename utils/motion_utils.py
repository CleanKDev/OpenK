from __future__ import annotations

import logging
import time
from numbers import Number
from typing import Any


def _extract_pos_values(data: dict[str, Any]) -> dict[str, float]:
    pos = {}
    for key, value in data.items():
        if not key.endswith(".pos"):
            continue
        if not isinstance(value, Number) or isinstance(value, bool):
            continue
        pos[key] = float(value)
    return pos


def build_home_action(action_features: dict[str, type], home_value: float = 0.0) -> dict[str, float]:
    return {key: home_value for key in action_features if key.endswith(".pos")}


def ramp_action(
    robot: object,
    start_action: dict[str, Any],
    target_action: dict[str, Any],
    *,
    duration_s: float,
    dt_s: float,
) -> None:
    if duration_s <= 0 or dt_s <= 0:
        robot.send_action(target_action)
        return

    start_pos = _extract_pos_values(start_action)
    target_pos = _extract_pos_values(target_action)
    if not target_pos:
        return

    steps = max(1, int(duration_s / dt_s))
    for i in range(1, steps + 1):
        alpha = i / steps
        action = {}
        for key, target_val in target_pos.items():
            start_val = start_pos.get(key, target_val)
            action[key] = (1.0 - alpha) * start_val + alpha * target_val
        robot.send_action(action)
        time.sleep(dt_s)


def move_home_and_disable_torque(
    robot: object,
    *,
    home_action: dict[str, Any],
    ramp_time_s: float,
    ramp_dt_s: float,
    settle_time_s: float,
) -> None:
    if not getattr(robot, "is_connected", False):
        return
    try:
        obs = robot.get_observation()
    except Exception:
        logging.exception("Failed to read observation for home move.")
        obs = {}

    ramp_action(
        robot,
        obs,
        home_action,
        duration_s=ramp_time_s,
        dt_s=ramp_dt_s,
    )
    time.sleep(max(0.0, settle_time_s))

    for attr in ("bus", "sts_bus"):
        bus = getattr(robot, attr, None)
        if bus is None:
            continue
        disable = getattr(bus, "disable_torque", None)
        if disable is None:
            continue
        try:
            disable()
        except Exception:
            logging.exception("Failed to disable torque on %s.", attr)
