#!/usr/bin/env python

"""
Utility to format action/observation motor states as a fixed-width CLI table.

The table shows motor names in the first column and aligns numeric values for
act/obs position and (optionally) velocity/torque in subsequent columns.

Example output:

--------------------------------------------
MOTOR        | act.Pos | obs.Pos | obs.Vel | obs.Tor
shoulder_pan |   12.34 |   12.31 |    0.05 |   -0.12
elbow_flex   |   -3.21 |   -3.19 |    0.01 |    0.00
grip         |   45.00 |   44.95 |    0.00 |    0.02

Missing values are rendered as "-".
"""

from __future__ import annotations

from numbers import Number
from typing import Any

_STATE_SUFFIXES = ("pos", "vel", "tor")


def format_motor_state_table(
    action: dict[str, Any],
    observation: dict[str, Any],
    *,
    precision: int = 3,
) -> tuple[str, int]:
    """
    Build a fixed-width table for action/observation motor states.

    Args:
        action: Dict with keys like "<motor>.<pos|vel>".
        observation: Dict with keys like "<motor>.<pos|vel|tor>".
        precision: Number of decimal places for numeric values.

    Returns:
        (table_text, line_count)
    """
    act_state, act_order = _extract_state(action)
    obs_state, obs_order = _extract_state(observation)

    motors = _merge_order(act_order, obs_order)
    if not motors:
        return "NO MOTOR DATA", 1

    has_act_vel = any("vel" in act_state.get(motor, {}) for motor in motors)

    columns: list[tuple[str, dict[str, dict[str, float]], str]] = [
        ("act.Pos", act_state, "pos"),
        ("obs.Pos", obs_state, "pos"),
    ]
    if has_act_vel:
        columns.append(("act.Vel", act_state, "vel"))
    columns.append(("obs.Vel", obs_state, "vel"))
    columns.append(("obs.Tor", obs_state, "tor"))

    name_header = "MOTOR"
    name_width = max(len(name_header), *(len(motor) for motor in motors))

    cell_values = _build_cells(motors, columns, precision)
    col_widths = [
        max(len(header), *(len(row[idx]) for row in cell_values))
        for idx, (header, _, _) in enumerate(columns)
    ]

    header_cells = [name_header.ljust(name_width)]
    for (header, _, _), width in zip(columns, col_widths):
        header_cells.append(header.rjust(width))
    header_line = " | ".join(header_cells)
    separator_line = "-" * len(header_line)

    lines = [separator_line, header_line]
    for motor, row in zip(motors, cell_values):
        row_cells = [motor.ljust(name_width)]
        for value, width in zip(row, col_widths):
            row_cells.append(value.rjust(width))
        lines.append(" | ".join(row_cells))

    return "\n".join(lines), len(lines)


def _extract_state(data: dict[str, Any]) -> tuple[dict[str, dict[str, float]], list[str]]:
    state: dict[str, dict[str, float]] = {}
    order: list[str] = []

    for key, value in data.items():
        if not isinstance(value, Number) or isinstance(value, bool):
            continue
        motor, suffix = _split_motor_key(key)
        if motor is None:
            continue
        if motor not in state:
            state[motor] = {}
            order.append(motor)
        state[motor][suffix] = float(value)

    return state, order


def _split_motor_key(key: str) -> tuple[str | None, str | None]:
    if "." not in key:
        return None, None
    motor, suffix = key.rsplit(".", 1)
    if suffix not in _STATE_SUFFIXES:
        return None, None
    return motor, suffix


def _merge_order(primary: list[str], secondary: list[str]) -> list[str]:
    merged = list(primary)
    for motor in secondary:
        if motor not in merged:
            merged.append(motor)
    return merged


def _build_cells(
    motors: list[str],
    columns: list[tuple[str, dict[str, dict[str, float]], str]],
    precision: int,
) -> list[list[str]]:
    rows: list[list[str]] = []
    for motor in motors:
        row_values: list[str] = []
        for _, state, suffix in columns:
            value = state.get(motor, {}).get(suffix)
            row_values.append(_format_value(value, precision))
        rows.append(row_values)
    return rows


def _format_value(value: float | None, precision: int) -> str:
    if value is None:
        return "-"
    return f"{value:.{precision}f}"
