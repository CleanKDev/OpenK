from contextlib import contextmanager
from typing import Dict, TypeAlias
from serial import Serial
from functools import cached_property
from dataclasses import dataclass
from enum import Enum
from pprint import pformat
from copy import deepcopy
from pathlib import Path  # noqa: F401
import draccus  # noqa: F401

import logging
import time
import math
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.utils import enter_pressed, move_cursor_up
from .DM_CAN import Control_Type, MotorControl, Motor, DM_variable

DEFAULT_BAUDRATE = 921600

NameOrID: TypeAlias = str | int
Value: TypeAlias = int | float
GainSpec: TypeAlias = float | dict[str, float] | None

DEFAULT_KP = 20.0
DEFAULT_KD = 1
DEFAULT_TORQUE_LIMIT = 10.0
DEFAULT_VELOCITY_LIMIT = 2.0
MIN_TELEMETRY_INTERVAL_S = 0.005

logger = logging.getLogger(__name__)

class MotorNormMode(str, Enum):
    NONE = "none"
    RANGE_0_100 = "range_0_100"
    RANGE_M100_100 = "range_m100_100"
    DEGREES = "degrees"
    CENTERING = "centering"

@dataclass
class MotorCalibration:
    id: int
    motor_offset: float
    range_min: float
    range_max: float

class DamiaoMotorsBus:
    """CAN bus helper that talks to DaMiao actuators through the vendor SDK."""
    def __init__(
        self,
        port: str,
        motors: Dict[str, Motor],
        baudrate: int = DEFAULT_BAUDRATE,
        control_type=Control_Type.MIT,
        calibration: dict[str, MotorCalibration] | None = None,
        motor_norm_mode: MotorNormMode = MotorNormMode.RANGE_M100_100,
        kp: GainSpec = None,
        kd: GainSpec = None,
        velocity_limit: GainSpec = None,
    ):
        self.port = port
        self.motors : Dict[str, Motor] = motors
        self.baudrate = baudrate
        self.control_type = control_type
        self.serial_device = Serial(port, baudrate)
        self._last_refresh_time: float | None = None #Track when motor telemetry was last refreshed so we can throttle polling
        self.calibration: dict[str, MotorCalibration] = calibration.copy() if calibration else {}
        self.motor_norm_mode = motor_norm_mode

        self.motorcontrol = MotorControl(self.serial_device)
        self.motorcontrol.serial_.close()
        for motor in self.motors.values():
            self.motorcontrol.addMotor(motor)
            logger.debug(f"Added motor {motor} to DamiaoMotorsBus on port {self.port}")
        self._has_calibration: bool = bool(self.calibration)
        self.kp = kp
        self.kd = kd
        self.velocity_limit = velocity_limit

    def _get_motor_names(self,):
        return list(self.motors.keys())

    @cached_property
    def motor_names(self):
        return self._get_motor_names()

    def connect(self):
        """Open the serial port and switch every attached motor to the chosen control mode."""
        if self.is_connected:
            raise DeviceAlreadyConnectedError(
                f"{self.__class__.__name__}('{self.port}') is already connected. Do not call `{self.__class__.__name__}.connect()` twice."
            )
        self.motorcontrol.serial_.open()
        logger.debug(f"{self.__class__.__name__} connected.")

        for motor in self.motors.values():
            try:
                self.motorcontrol.switchControlMode(motor, self.control_type)
                logger.info(f"Switched motor {motor} to {self.control_type} control mode")
                self.motorcontrol.enable(motor)
                logger.info(f"Enabled motor {motor}")
            except Exception as e:
                logger.warning(f"Failed to switch motor {motor} to {self.control_type} control mode: {e}")


    @property
    def is_connected(self) -> bool:
        return self.motorcontrol.serial_.is_open

    @property
    def is_calibrated(self) -> bool:
        return self._has_calibration and self.calibrated()

    def disconnect(self, disable_torque: bool = True) -> None:
        """Close the serial port (optionally disabling torque first).

        Args:
            disable_torque (bool, optional): If `True` (default) torque is disabled on every motor before
                closing the port. This can prevent damaging motors if they are left applying resisting torque
                after disconnect.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__}('{self.port}') is not connected. Try running `{self.__class__.__name__}.connect()` first."
            )

        if disable_torque:
            self.disable_torque()
        self.motorcontrol.serial_.close()
        logger.debug(f"{self.__class__.__name__} disconnected.")

    def disable_torque(self, motors: str | list[str] | None = None) -> None:
        """Disable torque for the selected motors (defaults to all)."""
        selected_motors = self._select_motors(motors)
        for motor in selected_motors.values():
            self.motorcontrol.disable(motor)

    def enable_torque(self, motors: str | list[str] | None = None) -> None:
        """Enable torque for the selected motors (defaults to all)."""
        selected_motors = self._select_motors(motors)
        for motor in selected_motors.values():
            self.motorcontrol.enable(motor)

    @contextmanager
    def torque_disabled(self, motors: str | list[str] | None = None):
        """Context manager that temporarily disables torque on selected motors."""
        self.disable_torque(motors)
        try:
            yield
        finally:
            self.enable_torque(motors)

    @contextmanager
    def compliant_mode(self, motors: str | list[str] | None = None):
        """Context manager that puts motors in compliant (passive) mode.
        
        Unlike torque_disabled, this keeps motors enabled so they still return
        position feedback, but with zero stiffness so they can be moved by hand.
        """
        selected_motors = self._select_motors(motors)
        try:
            # Send MIT commands with kp=0, kd=small to make motors compliant
            for motor_name, motor in selected_motors.items():
                self.motorcontrol.controlMIT(DM_Motor=motor, kp=0, kd=0.1, q=0, dq=0, tau=0)
            yield
        finally:
            # Re-enable normal control by sending a position command
            pass  # Motors will be controlled normally on next sync_write



    def sync_read(
        self,
        motors: str | list[str] | None = None,
        *,
        normalize: bool = True,
        num_retry: int = 0,
    ) -> dict[str, Value]:
        """Return the latest position/velocity/torque values for the requested motors.

        Args:
            motors (str | list[str] | None, optional): Selection of motors to query. `None`
                loads every motor. Defaults to `None`.
            normalize (bool, optional): If `True` (default) positions are expressed in the
                configured :class:`MotorNormMode`. Set to `False` to obtain raw radians.
            num_retry (int, optional): Reserved for future retries (unused for DaMiao).

        Returns:
            dict[str, Value]: Flat mapping ``<motor>.<pos|vel|tor>`` to the latest readings.
        """

        selected_motors = self._select_motors(motors)

        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__}('{self.port}') is not connected. You need to run `{self.__class__.__name__}.connect()`."
            )

        now = time.monotonic()
        needs_refresh = (
            self._last_refresh_time is None or (now - self._last_refresh_time) >= MIN_TELEMETRY_INTERVAL_S
        )
        # Refresh telemetry if the minimum interval elapsed
        if needs_refresh:
            for motor in selected_motors.values():
                self.motorcontrol.refresh_motor_status(motor)
            self.motorcontrol.recv()
            self._last_refresh_time = now

        res = {}

        position = {motor_name: motor.getPosition() for motor_name, motor in selected_motors.items()}

        if normalize:
            if not self._has_calibration:
                raise RuntimeError("Cannot normalize Damiao positions without calibration.")
            position = self.normalize_positions(position)

        res.update({f'{motor_name}.pos': pos for motor_name, pos in position.items()})
        res.update({f'{motor_name}.vel': motor.getVelocity() for motor_name, motor in selected_motors.items()})
        res.update({f'{motor_name}.tor': motor.getTorque()   for motor_name, motor in selected_motors.items()})

        return res

    def sync_write(
        self,
        action: dict[str, Value],
        normalize: bool = True,
    ) -> None:
        """Send desired position/velocity/torque commands (normalized by default) to the motors.

        Args:
            action (dict[str, Value]): Mapping ``<motor>.<pos|vel|tor>`` to desired values.
            normalize (bool, optional): If `True` (default) `.pos` entries are interpreted in the
                configured :class:`MotorNormMode`, otherwise raw radians are expected.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__}('{self.port}') is not connected. You need to run `{self.__class__.__name__}.connect()`."
            )

        targets = {}
        for motor_name, motor in self.motors.items():
            pos = action[f"{motor_name}.pos"]
            vel = action.get(f"{motor_name}.vel")
            tor = action.get(f"{motor_name}.tor")
            targets[motor_name] = {"motor": motor, "pos": pos, "vel": vel, "tor": tor}

        if normalize:
            if not self._has_calibration:
                raise RuntimeError("Cannot normalize Damiao positions without calibration.")
            normalized_positions = {motor_name: target["pos"] for motor_name, target in targets.items()}
            normalized_positions = self.unnormalize_positions(normalized_positions)
            for motor_name, raw_value in normalized_positions.items():
                targets[motor_name]["pos"] = raw_value

        if self.control_type == Control_Type.MIT:
            for motor_name, motor in self.motors.items():
                target = targets[motor_name]
                pos = target["pos"]
                vel = target.get("vel", 0.0) or 0.0
                kp = self._resolve_gain(self.kp, motor_name, DEFAULT_KP)
                kd = self._resolve_gain(self.kd, motor_name, DEFAULT_KD)
                self.motorcontrol.controlMIT(DM_Motor=motor, kp=kp, kd=kd, q=pos, dq=vel, tau=0.0)

        elif self.control_type == Control_Type.POS_VEL:
            for motor_name, motor in self.motors.items():
                pos = targets[motor_name]["pos"]
                vel_limit = self._resolve_gain(self.velocity_limit, motor_name, DEFAULT_VELOCITY_LIMIT)
                self.motorcontrol.control_Pos_Vel(Motor=motor, P_desired=pos, V_desired=vel_limit)

        else:
            raise NotImplementedError(f"Control type {self.control_type} is not implemented in DamiaoMotorsBus.sync_write()")

    @staticmethod
    def _resolve_gain(spec: GainSpec, motor_name: str, default: float) -> float:
        if spec is None:
            return float(default)
        if isinstance(spec, (int, float)):
            return float(spec)
        if isinstance(spec, dict):
            if motor_name not in spec:
                return float(default)
            value = spec[motor_name]
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise TypeError(f"Gain for '{motor_name}' must be numeric.")
            return float(value)
        raise TypeError(f"Unsupported gain spec for '{motor_name}': {spec!r}")



    def reset_offset(self,
        motors: str | list[str] | None = None
    ) -> dict[str, int]:
        """Reset the homing offset of several motors to zero and return the written offsets.

        Args:
            motors (str | list[str] | None, optional): Motors to reset. Defaults to all motors (`None`).
        Returns:
            dict[str, int]: Mapping *motor name → new offset*.
        """
        selected_motors = self._select_motors(motors)
        offset = {}

        for motor_name, motor in selected_motors.items():
            self.motorcontrol.set_zero_position(motor)

            time.sleep(0.05)  # small delay to ensure command is processed
            offset[motor_name] = self.motorcontrol.read_motor_param(motor, RID=DM_variable.m_off)

        return offset

    def read_offsets(self, motors: str | list[str] | None = None) -> dict[str, float]:
        """Read current homing offsets (m_off) from motors without writing."""
        selected_motors = self._select_motors(motors)
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__}('{self.port}') is not connected. You need to run `{self.__class__.__name__}.connect()`."
            )
        offsets: dict[str, float] = {}
        for motor_name, motor in selected_motors.items():
            current = self.motorcontrol.read_motor_param(motor, RID=DM_variable.m_off)
            if current is None:
                raise RuntimeError(f"Failed to read homing offset for motor {motor_name}")
            offsets[motor_name] = float(current)
        return offsets


    def record_ranges_of_motion(
        self,
        motors: str | list[str] | None = None,
        display_values: bool = True,
        poll_interval_s: float = 0.05,
    ) -> tuple[dict[str, float], dict[str, float]]:
        """Interactively record the min/max encoder values of each motor.

        Args:
            motors (str | list[str] | None, optional): Selection of motors to monitor. `None`
                uses all available motors.
            display_values (bool, optional): If `True` prints a rolling table of min/pos/max
                to the console. Defaults to `True`.
            poll_interval_s (float, optional): Delay between successive reads when logging to
                the console. Defaults to ``0.05`` seconds.

        Returns:
            tuple[dict[str, float], dict[str, float]]: Two dictionaries mapping *motor → min*
            and *motor → max* across the recorded trajectory.
        """
        selected_motors = self._select_motors(motors)

        if not selected_motors:
            return {}, {}

        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__}('{self.port}') is not connected. You need to run `{self.__class__.__name__}.connect()`."
            )

        mins: dict[str, float] = {}
        maxes: dict[str, float] = {}

        user_pressed_enter = False
        while not user_pressed_enter:
            positions = self._read_positions(selected_motors)

            for motor_name in selected_motors:
                if motor_name not in positions:
                    continue

                pos = positions[motor_name]
                if motor_name not in mins:
                    mins[motor_name] = pos
                    maxes[motor_name] = pos
                else:
                    mins[motor_name] = min(pos, mins[motor_name])
                    maxes[motor_name] = max(pos, maxes[motor_name])

            if display_values:
                print("\n-------------------------------------------")
                print(f"{'NAME':<15} | {'MIN':>9} | {'POS':>9} | {'MAX':>9}")
                for motor in selected_motors:
                    m_min = f"{mins[motor]:>9.4f}" if motor in mins else "      ---"
                    m_pos = f"{positions[motor]:>9.4f}" if motor in positions else "  TIMEOUT"
                    m_max = f"{maxes[motor]:>9.4f}" if motor in maxes else "      ---"
                    print(f"{motor:<15} | {m_min} | {m_pos} | {m_max}")

            if enter_pressed():
                user_pressed_enter = True
            else:
                if display_values:
                    move_cursor_up(len(selected_motors) + 3)
                time.sleep(poll_interval_s)

        # Check if all motors were reached at least once
        missing_readings = [m for m in selected_motors if m not in mins]
        if missing_readings:
            raise RuntimeError(f"Could not get any reading for motors: {missing_readings}. Check connections.")

        same_min_max = [motor for motor in selected_motors if mins[motor] == maxes[motor]]
        if same_min_max:
            raise ValueError(f"Some motors have the same min and max values:\n{pformat(same_min_max)}")

        return mins, maxes

    def read_calibration(self) -> dict[str, MotorCalibration]:
        """Return a deep copy of cached calibration entries."""
        return deepcopy(self.calibration)

    def write_calibration(self, calibration_dict: dict[str, MotorCalibration], cache: bool = True) -> None:
        """Update cached calibration offsets (m_off is read-only and not written)."""
        unknown = set(calibration_dict) - set(self.motors)
        if unknown:
            raise KeyError(f"Unknown motors in calibration: {unknown}")
        if cache:
            self.calibration = deepcopy(calibration_dict)
            self._has_calibration = True


    def reset_calibration(self) -> None:
        """Clear cached calibration data (does not write m_off)."""
        self.calibration = {}
        self._has_calibration = False

    def normalize_positions(self, positions: dict[str, float]) -> dict[str, float]:
        """Convert raw radian positions into user-facing units based on MotorNormMode."""
        normalized = {}
        for motor_name, value in positions.items():
            norm_mode = self.motor_norm_mode
            min_, max_ = self._get_calibration_range(motor_name)
            if norm_mode is MotorNormMode.NONE:
                normalized[motor_name] = self._clamp(value, min_, max_)
            elif norm_mode is MotorNormMode.DEGREES:
                normalized[motor_name] = math.degrees(self._clamp(value, min_, max_))
            elif norm_mode is MotorNormMode.CENTERING:
                center = (min_ + max_) / 2
                bounded = self._clamp(value, min_, max_)
                normalized[motor_name] = bounded - center
            else:
                bounded = self._clamp(value, min_, max_)
                span = max_ - min_
                if span == 0:
                    raise ValueError(f"Invalid calibration for motor '{motor_name}': min and max are equal.")
                if norm_mode is MotorNormMode.RANGE_M100_100:
                    normalized[motor_name] = ((bounded - min_) / span) * 200.0 - 100.0
                elif norm_mode is MotorNormMode.RANGE_0_100:
                    normalized[motor_name] = ((bounded - min_) / span) * 100.0
                else:
                    raise NotImplementedError(f"Normalization for {norm_mode} not implemented.")
        return normalized

    def unnormalize_positions(self, positions: dict[str, float]) -> dict[str, float]:
        """Convert normalized user units back into raw radians."""
        unnormalized = {}
        for motor_name, value in positions.items():
            norm_mode = self.motor_norm_mode
            min_, max_ = self._get_calibration_range(motor_name)
            if norm_mode is MotorNormMode.NONE:
                unnormalized[motor_name] = self._clamp(value, min_, max_)
                continue
            if norm_mode is MotorNormMode.DEGREES:
                radians_val = math.radians(value)
                unnormalized[motor_name] = self._clamp(radians_val, min_, max_)
                continue
            span = max_ - min_
            if span == 0:
                raise ValueError(f"Invalid calibration for motor '{motor_name}': min and max are equal.")

            if norm_mode is MotorNormMode.RANGE_M100_100:
                bounded = self._clamp(value, -100.0, 100.0)
                unnormalized[motor_name] = ((bounded + 100.0) / 200.0) * span + min_
            elif norm_mode is MotorNormMode.RANGE_0_100:
                bounded = self._clamp(value, 0.0, 100.0)
                unnormalized[motor_name] = (bounded / 100.0) * span + min_
            elif norm_mode is MotorNormMode.CENTERING:
                half_span = span / 2
                bounded = self._clamp(value, -half_span, half_span)
                center = (min_ + max_) / 2
                unnormalized[motor_name] = bounded + center
            else:
                raise NotImplementedError(f"Unnormalization for {norm_mode} not implemented.")
        return unnormalized

    def _select_motors(self, motors: str | list[str] | None) -> dict[str, Motor]:
        """Return a mapping of motor names to Motor objects based on user selection."""
        if motors is None:
            return self.motors
        if isinstance(motors, str):
            return {motors: self.motors[motors]}
        if isinstance(motors, list):
            return {motor_name: self.motors[motor_name] for motor_name in motors}
        raise TypeError(motors)

    def _read_positions(self, motors: dict[str, Motor]) -> dict[str, float]:
        """Fetch latest position readings for the provided motors.
        
        Uses MIT control with zero gains to ensure position feedback is returned.
        This is more reliable than refresh_motor_status during calibration.
        """
        if not motors:
            return {}
        # Send MIT command with zero torque to get position feedback
        for motor in motors.values():
            self.motorcontrol.controlMIT(DM_Motor=motor, kp=0, kd=0.1, q=0, dq=0, tau=0)
        self.motorcontrol.recv()
        return {motor_name: motor.getPosition() for motor_name, motor in motors.items()}

    def _get_calibration_range(self, motor_name: str) -> tuple[float, float]:
        if motor_name not in self.calibration:
            raise RuntimeError(f"No calibration available for motor '{motor_name}'.")
        calib = self.calibration[motor_name]
        return calib.range_min, calib.range_max

    def calibrated(self, motors: str | list[str] | None = None) -> bool:
        """Return True when calibration data exist for the selected motors."""
        selected_motors = self._select_motors(motors)
        return all(motor_name in self.calibration for motor_name in selected_motors)

    @staticmethod
    def _clamp(value: float, min_: float, max_: float) -> float:
        return max(min_, min(max_, value))

    def check_offset(self,motors: str | list[str] | None =None) -> bool:

        """Check if the selected motors have same offset as cached calibration."""

        if not self._has_calibration:
            return False
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__}('{self.port}') is not connected. You need to run `{self.__class__.__name__}.connect()`."
            )
        selected_motors = self._select_motors(motors)
        for motor_name, motor in selected_motors.items():
            cached_offset = self.calibration.get(motor_name)
            if not cached_offset:
                return False
            current_offset = self.motorcontrol.read_motor_param(motor, RID=DM_variable.m_off)
            # Fix: Handle communication failure
            if current_offset is None:
                logging.warning(f"Could not read offset for motor {motor_name} (ID {motor.SlaveID}). Assuming check failed.")
                return False

            if abs(cached_offset.motor_offset - current_offset) > 1e-4:
                return False
        return True

    def get_offset_mismatches(
        self,
        motors: str | list[str] | None = None,
        *,
        tolerance: float = 1e-4,
    ) -> dict[str, tuple[float | None, float | None]]:
        """Return mismatched m_off values as {motor_name: (cached, current)}."""
        if not self._has_calibration:
            return {}
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__}('{self.port}') is not connected. You need to run `{self.__class__.__name__}.connect()`."
            )
        selected_motors = self._select_motors(motors)
        mismatches: dict[str, tuple[float | None, float | None]] = {}
        for motor_name, motor in selected_motors.items():
            cached = self.calibration.get(motor_name)
            cached_value = cached.motor_offset if cached is not None else None
            current = self.motorcontrol.read_motor_param(motor, RID=DM_variable.m_off)
            if current is None:
                logging.warning(
                    f"Could not read offset for motor {motor_name} (ID {motor.SlaveID})."
                )
                mismatches[motor_name] = (cached_value, None)
                continue
            if cached_value is None or abs(cached_value - current) > tolerance:
                mismatches[motor_name] = (cached_value, current)
        return mismatches
