#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path
from typing import Any

import draccus

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.motors import Motor as StsMotor
from lerobot.motors import MotorCalibration as StsMotorCalibration
from lerobot.motors import MotorNormMode as FeetechMotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS, TELEOPERATORS
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from lerobot.robots import Robot
from lerobot.robots.utils import ensure_safe_goal_position

from ..damiao.DM_CAN import Control_Type, DM_Motor_Type, Motor as DamiaoMotor
from ..damiao.damiao import DamiaoMotorsBus, MotorCalibration as DamiaoMotorCalibration
from ..damiao.damiao import MotorNormMode as DamiaoMotorNormMode
from .config_openk_follower2 import OpenKFollower2Config, DamiaoMotorSpec, StsMotorSpec

logger = logging.getLogger(__name__)


@dataclass
class CombinedCalibration:
    damiao: dict[str, DamiaoMotorCalibration] = field(default_factory=dict)
    sts: dict[str, StsMotorCalibration] = field(default_factory=dict)


def _get_spec_value(spec: object, key: str):
    if hasattr(spec, key):
        return getattr(spec, key)
    if isinstance(spec, dict) and key in spec:
        return spec[key]
    raise KeyError(f"Missing '{key}' in motor spec: {spec}")


def _resolve_damiao_motor_type(value: object) -> DM_Motor_Type:
    if isinstance(value, DM_Motor_Type):
        return value
    if isinstance(value, str):
        try:
            return DM_Motor_Type[value]
        except KeyError as exc:
            raise ValueError(f"Unknown Damiao motor type: {value}") from exc
    if isinstance(value, int):
        return DM_Motor_Type(value)
    raise TypeError(f"Unsupported Damiao motor type value: {value!r}")


def _build_damiao_motors(specs: list[DamiaoMotorSpec | dict[str, object]]) -> dict[str, DamiaoMotor]:
    motors: dict[str, DamiaoMotor] = {}
    for spec in specs:
        name = str(_get_spec_value(spec, "name"))
        motor_type = _resolve_damiao_motor_type(_get_spec_value(spec, "motor_type"))
        slave_id = int(_get_spec_value(spec, "slave_id"))
        master_id = int(_get_spec_value(spec, "master_id"))
        motors[name] = DamiaoMotor(motor_type, slave_id, master_id)
    return motors


def _build_sts_motors(
    specs: list[StsMotorSpec | dict[str, object]],
    norm_mode: FeetechMotorNormMode,
) -> dict[str, StsMotor]:
    motors: dict[str, StsMotor] = {}
    for spec in specs:
        name = str(_get_spec_value(spec, "name"))
        motor_id = int(_get_spec_value(spec, "motor_id"))
        model = str(_get_spec_value(spec, "model"))
        motors[name] = StsMotor(motor_id, model, norm_mode)
    return motors


class OpenKFollower2(Robot):
    """OpenK follower variant using Feetech STS for wrist_flex/grip."""

    config_class = OpenKFollower2Config
    name = "openk_follower2"

    def __init__(self, config: OpenKFollower2Config):
        super().__init__(config)

        self.config = config
        self.id = self.config.id
        if self.config.calibration_dir is not None:
            self.calibration_dir = Path(self.config.calibration_dir)
        elif self.config.calibration_root is not None:
            self.calibration_dir = Path(self.config.calibration_root) / ROBOTS / self.name
        else:
            self.calibration_dir = HF_LEROBOT_CALIBRATION / TELEOPERATORS / self.name
        self.calibration_dir.mkdir(parents=True, exist_ok=True)
        self.calibration_fpath = self.calibration_dir / f"{self.id}.json"
        self.calibration: dict[str, DamiaoMotorCalibration] = {}
        self.sts_calibration: dict[str, StsMotorCalibration] = {}
        self._sts_has_calibration = False
        self._damiao_offset_mismatches: dict[str, tuple[float | None, float | None]] = {}
        if self.calibration_fpath.is_file():
            self._load_calibration()

        self.config = config
        motor_norm_mode: DamiaoMotorNormMode = config.motor_norm_mode
        control_type = config.control_type
        damiao_motors = _build_damiao_motors(config.damiao_motors)
        self.bus = DamiaoMotorsBus(
            port=self.config.port,
            motors=damiao_motors,
            calibration=self.calibration,
            motor_norm_mode=motor_norm_mode,
            control_type=control_type,
            kp=self.config.damiao_kp,
            kd=self.config.damiao_kd,
            velocity_limit=self.config.damiao_velocity_limit,
        )
        sts_motors = _build_sts_motors(config.sts_motors, self.config.sts_motor_norm_mode)
        self.sts_bus = FeetechMotorsBus(
            port=self.config.sts_port,
            motors=sts_motors,
            calibration=self.sts_calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        res: dict[str, type] = {}
        res.update({f"{motor}.pos": float for motor in self.bus.motor_names})
        res.update({f"{motor}.pos": float for motor in self.sts_bus.motors})
        res.update({f"{motor}.vel": float for motor in self.bus.motor_names})
        res.update({f"{motor}.tor": float for motor in self.bus.motor_names})
        return res

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @property
    def _motors_ft2(self) -> dict[str, type]:
        res: dict[str, type] = {}
        res.update({f"{motor}.pos": float for motor in self.bus.motor_names})
        res.update({f"{motor}.pos": float for motor in self.sts_bus.motors})
        return res

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft2

    def _load_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        logger.info(f"Loading calibration from {fpath}")
        with open(fpath) as f, draccus.config_type("json"):
            combined = draccus.load(CombinedCalibration, f)

        self.calibration = combined.damiao
        self.sts_calibration = combined.sts
        self._sts_has_calibration = bool(self.sts_calibration)
        logger.info(f"Calibration loaded: Damiao={bool(self.calibration)}, STS={self._sts_has_calibration}")

    def _save_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        combined = CombinedCalibration(damiao=self.calibration, sts=self.sts_calibration)
        with open(fpath, "w") as f, draccus.config_type("json"):
            draccus.dump(combined, f, indent=2)

    @property
    def is_connected(self) -> bool:
        return (
            self.bus.is_connected
            and self.sts_bus.is_connected
            and all(cam.is_connected for cam in self.cameras.values())
        )

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated and self.sts_bus.is_calibrated

    def connect(self, calibrate: bool = True) -> None:
        """Open the motor buses and optionally run calibration."""
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        logger.info("Damiao connected.")
        self.sts_bus.connect(handshake=False)

        if self.calibration:
            self._damiao_offset_mismatches = self.bus.get_offset_mismatches()
            for motor_name, (cached, current) in self._damiao_offset_mismatches.items():
                logger.warning(
                    "Damiao %s offset mismatch: cached=%s, current=%s.",
                    motor_name,
                    f"{cached:.4f}" if cached is not None else "None",
                    f"{current:.4f}" if current is not None else "None",
                )

        self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    def calibrate(self) -> None:
        """Interactively record offsets/ranges of motion and persist them."""
        if self.calibration and self.sts_calibration:
            if self.bus.is_connected:
                self._damiao_offset_mismatches = self.bus.get_offset_mismatches()
            if self._damiao_offset_mismatches:
                logger.warning(
                    "Damiao m_off mismatch detected. Full calibration is required to proceed."
                )
            else:
                user_input = input(
                    f"Press ENTER to use provided calibration file associated with the id {self.id}, "
                    "or type 'c' and press ENTER to run full calibration: "
                )
                if user_input.strip().lower() != "c":
                    logger.info(f"Using calibration file associated with the id {self.id}")
                    # NOTE: write_calibration is cache-only (m_off is read-only), so torque_disabled is unnecessary.
                    # Kept commented for clarity in case hardware writes are reintroduced later.
                    # with self.bus.torque_disabled():
                    self.bus.write_calibration(self.calibration)
                    with self.sts_bus.torque_disabled():
                        self.sts_bus.write_calibration(self.sts_calibration)
                    return
        else:
            print(f"Calibration file missing for {self.id} at {self.calibration_fpath}")
            user_input = input("Would you like to run calibration now? [y/N]: ")
            if user_input.strip().lower() != "y":
                print("Exiting as calibration is required to continue.")
                exit(0)

        logger.info(f"\nRunning calibration of {self}")

        with self.bus.torque_disabled():
            user_input = input(
                f"Move {self} to the middle of its range of motion and press ENTER to set zero "
                "(writes m_off), or type 's' and press ENTER to skip homing: "
            )
            if user_input.strip().lower() in {"s", "skip"}:
                homing_offsets = self.bus.read_offsets()
            else:
                homing_offsets = self.bus.reset_offset()

            print(
                "Move all joints sequentially through their entire ranges "
                "of motion.\nRecording positions. Press ENTER to stop..."
            )
            range_mins, range_maxes = self.bus.record_ranges_of_motion()

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            if homing_offsets.get(motor) is None:
                raise ValueError(f"Failed to get homing offset for motor {motor}")
            if range_mins.get(motor) is None:
                raise ValueError(f"Failed to get range min for motor {motor}")
            if range_maxes.get(motor) is None:
                raise ValueError(f"Failed to get range max for motor {motor}")

            self.calibration[motor] = DamiaoMotorCalibration(
                id=int(m.SlaveID),
                motor_offset=float(homing_offsets[motor]),
                range_min=float(range_mins[motor]),
                range_max=float(range_maxes[motor]),
            )

        # NOTE: write_calibration is cache-only (m_off is read-only), so torque_disabled is unnecessary.
        # Kept commented for clarity in case hardware writes are reintroduced later.
        # with self.bus.torque_disabled():
        self.bus.write_calibration(self.calibration)

        self.sts_bus.disable_torque()
        for motor in self.sts_bus.motors:
            self.sts_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move {self} (STS) to the middle of its range of motion and press ENTER....")
        homing_offsets = self.sts_bus.set_half_turn_homings()

        print(
            "Move STS joints sequentially through their entire ranges "
            "of motion.\nRecording positions. Press ENTER to stop..."
        )
        sts_range_mins, sts_range_maxes = self.sts_bus.record_ranges_of_motion()
        self.sts_calibration = {}
        for motor, m in self.sts_bus.motors.items():
            self.sts_calibration[motor] = StsMotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=int(homing_offsets[motor]),
                range_min=int(sts_range_mins[motor]),
                range_max=int(sts_range_maxes[motor]),
            )
        with self.sts_bus.torque_disabled():
            self.sts_bus.write_calibration(self.sts_calibration)
        self._sts_has_calibration = True
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        """Apply post-connection tweaks for both buses."""
        # Damiao: no-op for now

        # Feetech STS PID parameters similar to the STS leader.
        self.sts_bus.disable_torque()
        self.sts_bus.configure_motors()
        for motor in self.sts_bus.motors:
            self.sts_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            self.sts_bus.write("P_Coefficient", motor, 16)
            self.sts_bus.write("I_Coefficient", motor, 0)
            self.sts_bus.write("D_Coefficient", motor, 32)
        self.sts_bus.disable_torque()

    def get_observation(self) -> dict[str, Any]:
        """Return the latest motor state and camera frames."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        motor_state = self.bus.sync_read()
        obs_dict = motor_state.copy()
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read Damiao state: {dt_ms:.1f}ms")

        start = time.perf_counter()
        sts_state = self.sts_bus.sync_read("Present_Position", normalize=self._sts_has_calibration, num_retry=3)
        obs_dict.update({f"{motor}.pos": val for motor, val in sts_state.items()})
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read STS state: {dt_ms:.1f}ms")

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command the arm to move to a target joint configuration."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        time_start = time.perf_counter()

        goal_positions = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}
        damiao_goal_positions = {
            motor: val for motor, val in goal_positions.items() if motor in self.bus.motor_names
        }
        sts_goal_positions = {
            motor: val for motor, val in goal_positions.items() if motor in self.sts_bus.motors
        }

        if self.config.max_relative_target is not None and damiao_goal_positions:
            present_pos = self.bus.sync_read()
            goal_present_pos = {
                motor: (g_pos, present_pos[f"{motor}.pos"]) for motor, g_pos in damiao_goal_positions.items()
            }
            damiao_goal_positions = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        command = action.copy()
        for motor, val in damiao_goal_positions.items():
            command[f"{motor}.pos"] = val
            if self.bus.control_type == Control_Type.MIT and f"{motor}.vel" not in command:
                command[f"{motor}.vel"] = 0.0

        self.bus.sync_write(command)
        if sts_goal_positions:
            self.sts_bus.sync_write("Goal_Position", sts_goal_positions, normalize=self._sts_has_calibration)
        dt_ms = (time.perf_counter() - time_start) * 1e3
        logger.debug(f"{self} sent action: {dt_ms:.1f}ms")
        return command

    def disconnect(self):
        """Gracefully disconnect the robot and attached cameras."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        if self.sts_bus.is_connected:
            self.sts_bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
