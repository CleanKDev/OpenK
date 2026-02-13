import json
import logging
from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
from functools import cached_property
from pathlib import Path
import time
from typing import Dict, TypeAlias

from serial import Serial

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.utils import enter_pressed, move_cursor_up

from .STS_CAN import StsController, StsMotor

logger = logging.getLogger(__name__)

DEFAULT_BAUDRATE = 921600
DEFAULT_CONTROLLER_ID = 0xFF00

NameOrID: TypeAlias = str | int
Value: TypeAlias = int | float


class StsNormMode(str, Enum):
    NONE = "none"
    RANGE_0_100 = "range_0_100"
    RANGE_M100_100 = "range_m100_100"


@dataclass
class StsMotorCalibration:
    id: int
    offset: float
    range_min: float
    range_max: float


class StsMotorBus:
    """CAN bus helper for STS actuators, matching the lerobot motor bus API."""

    def __init__(
        self,
        port: str,
        motors: Dict[str, StsMotor],
        *,
        controller_id: int = DEFAULT_CONTROLLER_ID,
        baudrate: int = DEFAULT_BAUDRATE,
        calibration: dict[str, StsMotorCalibration] | None = None,
        norm_mode: StsNormMode = StsNormMode.RANGE_M100_100,
    ):
        self.port = port
        self.controller_id = controller_id
        self.motors = motors
        self.baudrate = baudrate
        self.calibration: dict[str, StsMotorCalibration] = calibration.copy() if calibration else {}
        self._has_calibration: bool = bool(calibration)
        self.norm_mode = norm_mode

        self.serial_device = Serial(port, baudrate)
        self._controller = StsController(self.serial_device)

    @cached_property
    def motor_names(self) -> list[str]:
        return list(self.motors.keys())

    @property
    def is_connected(self) -> bool:
        return self.serial_device.is_open

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(
                f"{self.__class__.__name__}('{self.port}') is already connected. Do not call `{self.__class__.__name__}.connect()` twice."
            )
        self.serial_device.open()
        logger.debug("%s connected.", self.__class__.__name__)

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__}('{self.port}') is not connected. Try running `{self.__class__.__name__}.connect()` first."
        )
        self.serial_device.close()
        logger.debug("%s disconnected.", self.__class__.__name__)

    def sync_read(
        self,
        motors: NameOrID | list[NameOrID] | None = None,
        *,
        normalize: bool = True,
    ) -> dict[str, Value]:
        selected = self._select_motors(motors)
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__}('{self.port}') is not connected. You need to run `{self.__class__.__name__}.connect()`."
            )

        res: dict[str, Value] = {}
        for motor_name, motor in selected.items():
            position = self._controller.read_position(self.controller_id, motor.id)
            if position is not None:
                res[f"{motor_name}.pos"] = position

        if normalize:
            if not self._has_calibration:
                raise RuntimeError("Cannot normalize STS positions without calibration.")
            positions = {k.split(".")[0]: v for k, v in res.items()}
            positions = self.normalize_positions(positions)
            res = {f"{name}.pos": value for name, value in positions.items()}
        return res

    def sync_write(self, action: dict[str, Value], *, normalize: bool = True) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__}('{self.port}') is not connected. You need to run `{self.__class__.__name__}.connect()`."
            )

        targets = {}
        for motor_name, motor in self.motors.items():
            targets[motor_name] = action[f"{motor_name}.pos"]

        if normalize:
            if not self._has_calibration:
                raise RuntimeError("Cannot unnormalize STS positions without calibration.")
            targets = self.unnormalize_positions(targets)

        for motor_name, motor in self.motors.items():
            angle = int(targets[motor_name])
            self._controller.write_position(self.controller_id, motor.id, angle)

    def read_calibration(self) -> dict[str, StsMotorCalibration]:
        return deepcopy(self.calibration)

    def write_calibration(self, calibration: dict[str, StsMotorCalibration], cache: bool = True) -> None:
        unknown = set(calibration) - set(self.motors)
        if unknown:
            raise KeyError(f"Unknown motors in calibration: {unknown}")
        if cache:
            self.calibration = deepcopy(calibration)
            self._has_calibration = True

    def record_ranges_of_motion(
        self,
        motors: NameOrID | list[NameOrID] | None = None,
        display_values: bool = True,
        poll_interval_s: float = 0.05,
    ) -> tuple[dict[str, float], dict[str, float]]:
        """Interactively record min/max positions for selected motors."""
        selected = self._select_motors(motors)
        if not selected:
            return {}, {}
        if not self.is_connected:
            raise DeviceNotConnectedError(
                f"{self.__class__.__name__}('{self.port}') is not connected. You need to run `{self.__class__.__name__}.connect()`."
            )

        mins: dict[str, float] = {}
        maxes: dict[str, float] = {}

        user_pressed_enter = False
        while not user_pressed_enter:
            positions = self._read_positions(selected)

            for motor_name in selected:
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
                for motor in selected:
                    m_min = f"{mins[motor]:>9.4f}" if motor in mins else "      ---"
                    m_pos = f"{positions[motor]:>9.4f}" if motor in positions else "  TIMEOUT"
                    m_max = f"{maxes[motor]:>9.4f}" if motor in maxes else "      ---"
                    print(f"{motor:<15} | {m_min} | {m_pos} | {m_max}")

            if enter_pressed():
                user_pressed_enter = True
            else:
                if display_values:
                    move_cursor_up(len(selected) + 3)
                time.sleep(poll_interval_s)

        # Check if all motors were reached at least once
        missing_readings = [m for m in selected if m not in mins]
        if missing_readings:
            raise RuntimeError(f"Could not get any reading for motors: {missing_readings}. Check connections.")

        same_min_max = [motor for motor in selected if mins[motor] == maxes[motor]]
        if same_min_max:
            raise ValueError(f"Some motors have the same min and max values: {same_min_max}")

        return mins, maxes

    def save_calibration(self, fpath: str | Path) -> None:
        """Persist calibration to disk (JSON)."""
        path = Path(fpath)
        path.parent.mkdir(parents=True, exist_ok=True)
        serializable = {name: vars(calib) for name, calib in self.calibration.items()}
        with path.open("w", encoding="utf-8") as f:
            json.dump(serializable, f, ensure_ascii=False, indent=2)

    def load_calibration(self, fpath: str | Path, cache: bool = True) -> dict[str, StsMotorCalibration]:
        path = Path(fpath)
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        loaded = {name: StsMotorCalibration(**fields) for name, fields in data.items()}
        if cache:
            self.calibration = deepcopy(loaded)
            self._has_calibration = True
        return loaded

    def reset_calibration(self) -> None:
        self.calibration = {}
        self._has_calibration = False

    def normalize_positions(self, positions: dict[str, float]) -> dict[str, float]:
        normalized = {}
        for motor_name, value in positions.items():
            min_, max_, offset = self._get_calibration_range(motor_name)
            shifted = self._clamp(value - offset, min_, max_)
            span = max_ - min_
            if span == 0:
                raise ValueError(f"Invalid calibration for motor '{motor_name}': min and max are equal.")
            if self.norm_mode is StsNormMode.NONE:
                normalized[motor_name] = shifted
            elif self.norm_mode is StsNormMode.RANGE_M100_100:
                normalized[motor_name] = ((shifted - min_) / span) * 200.0 - 100.0
            elif self.norm_mode is StsNormMode.RANGE_0_100:
                normalized[motor_name] = ((shifted - min_) / span) * 100.0
            else:
                raise NotImplementedError(f"Normalization for {self.norm_mode} not implemented.")
        return normalized

    def unnormalize_positions(self, positions: dict[str, float]) -> dict[str, float]:
        unnormalized = {}
        for motor_name, value in positions.items():
            min_, max_, offset = self._get_calibration_range(motor_name)
            span = max_ - min_
            if span == 0:
                raise ValueError(f"Invalid calibration for motor '{motor_name}': min and max are equal.")
            if self.norm_mode is StsNormMode.NONE:
                shifted = self._clamp(value, min_, max_)
            elif self.norm_mode is StsNormMode.RANGE_M100_100:
                bounded = self._clamp(value, -100.0, 100.0)
                shifted = ((bounded + 100.0) / 200.0) * span + min_
            elif self.norm_mode is StsNormMode.RANGE_0_100:
                bounded = self._clamp(value, 0.0, 100.0)
                shifted = (bounded / 100.0) * span + min_
            else:
                raise NotImplementedError(f"Unnormalization for {self.norm_mode} not implemented.")
            unnormalized[motor_name] = shifted + offset
        return unnormalized

    def _select_motors(self, motors: NameOrID | list[NameOrID] | None) -> dict[str, StsMotor]:
        if motors is None:
            return self.motors
        if isinstance(motors, str) and motors in self.motors:
            return {motors: self.motors[motors]}
        if isinstance(motors, int):
            for name, motor in self.motors.items():
                if motor.id == motors:
                    return {name: motor}
            raise KeyError(f"Unknown motor id: {motors}")
        if isinstance(motors, list):
            selected: dict[str, StsMotor] = {}
            for entry in motors:
                if isinstance(entry, str):
                    selected[entry] = self.motors[entry]
                elif isinstance(entry, int):
                    match = next((name for name, motor in self.motors.items() if motor.id == entry), None)
                    if match is None:
                        raise KeyError(f"Unknown motor id: {entry}")
                    selected[match] = self.motors[match]
                else:
                    raise KeyError(f"Unknown motor selection: {entry}")
            return selected
        raise KeyError(f"Unknown motor selection: {motors}")

    def _get_calibration_range(self, motor_name: str) -> tuple[float, float, float]:
        if motor_name not in self.calibration:
            raise RuntimeError(f"No calibration available for motor '{motor_name}'.")
        calib = self.calibration[motor_name]
        min_ = calib.range_min - calib.offset
        max_ = calib.range_max - calib.offset
        return min_, max_, calib.offset

    @staticmethod
    def _clamp(value: float, min_: float, max_: float) -> float:
        return max(min_, min(max_, value))

    def _read_positions(self, motors: dict[str, StsMotor]) -> dict[str, float]:
        """Fetch latest position readings for the provided motors."""
        if not motors:
            return {}
        positions = {}
        for motor_name, motor in motors.items():
            pos = self._controller.read_position(self.controller_id, motor.id)
            if pos is None:
                continue
            positions[motor_name] = float(pos)
        return positions
