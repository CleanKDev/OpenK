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
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, TELEOPERATORS

from ..damiao.DM_CAN import DM_Motor_Type, Motor
from ..damiao.damiao import DamiaoMotorsBus, MotorCalibration
from ..sts.motorbus import StsMotorBus, StsMotorCalibration
from ..sts.STS_CAN import StsMotor

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from lerobot.robots import Robot
from lerobot.robots.utils import ensure_safe_goal_position
from .config_openk_follower import OpenKFollowerConfig

logger = logging.getLogger(__name__)


@dataclass
class CombinedCalibration:
    damiao: dict[str, MotorCalibration] = field(default_factory=dict)
    sts: dict[str, StsMotorCalibration] = field(default_factory=dict)


class OpenKFollower(Robot):
    """Robot wrapper around the Damiao-powered OpenK follower arm."""

    config_class = OpenKFollowerConfig
    name = "openk_follower"

    def __init__(self, config: OpenKFollowerConfig):
        super().__init__(config)

        self.config = config
        self.id = self.config.id
        self.calibration_dir = (
            HF_LEROBOT_CALIBRATION / TELEOPERATORS / self.name
            if self.config.calibration_dir is None
            else Path(self.config.calibration_dir)
        )
        self.calibration_dir.mkdir(parents=True, exist_ok=True)
        self.calibration_fpath = self.calibration_dir / f"{self.id}.json"
        self.calibration: dict[str, MotorCalibration] = {}
        self.sts_calibration: dict[str, StsMotorCalibration] = {}
        self._sts_has_calibration = False
        if self.calibration_fpath.is_file():
            self._load_calibration()

        self.config = config
        motor_norm_mode = config.motor_norm_mode
        sts_norm_mode = config.sts_motor_norm_mode
        control_type = config.control_type
        self.bus = DamiaoMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan" : Motor(DM_Motor_Type.DM6006,0x01,0x15),
                "shoulder_lift": Motor(DM_Motor_Type.DM6006,0x02,0x15),
                "shoulder_roll": Motor(DM_Motor_Type.DM4310,0x03,0x15),
                "elbow_flex"   : Motor(DM_Motor_Type.DM4310,0x04,0x15),
                "wrist_roll"   : Motor(DM_Motor_Type.DM4310,0x05,0x15),
            },
            calibration=self.calibration,
            motor_norm_mode=motor_norm_mode,
            control_type=control_type,
        )
        self.sts_bus = StsMotorBus(
            port=self.config.sts_port,
            motors={
                "grip": StsMotor(1),
                "wrist_flex": StsMotor(2),
            },
            calibration=self.sts_calibration,
            norm_mode=sts_norm_mode,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        res = {}
        res.update({f"{motor}.pos": float for motor in self.bus.motor_names})
        res.update({f"{motor}.pos": float for motor in self.sts_bus.motor_names})
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
        res = {}
        res.update({f"{motor}.pos": float for motor in self.bus.motor_names})
        res.update({f"{motor}.pos": float for motor in self.sts_bus.motor_names})
        return res

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft2

    def _load_calibration(self, fpath: Path | None = None) -> None:
        """
        Helper to load calibration data from the specified file.

        Args:
            fpath (Path | None): Optional path to the calibration file. Defaults to `self.calibration_fpath`.
        """
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath) as f, draccus.config_type("json"):
            combined = draccus.load(CombinedCalibration, f)

        self.calibration = combined.damiao
        self.sts_calibration = combined.sts
        self._sts_has_calibration = bool(self.sts_calibration)

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


    def connect(self, calibrate: bool = True) -> None:
        """Open the motor bus and optionally run calibration."""
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        self.sts_bus.connect()

        requires_calibration = calibrate  # フラグが True なら絶対に対話モードへ

        if not requires_calibration:
            if not self.calibration:
                logger.info("キャリブレーションファイルなし，あるいはキーの不一致。")
                requires_calibration = True
            elif not self.sts_calibration:
                logger.info("STSキャリブレーションファイルなし。")
                requires_calibration = True
            elif not self.bus.check_offset():
                logger.warning("モータのオフセットとファイルが不一致。")
                requires_calibration = True
            else:
                logger.info("キャッシュ済みキャリブレーションがそのまま使えます。")

        if requires_calibration:
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()


        self.configure()
        logger.info(f"{self} connected.")


    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        """Interactively record offsets/ranges of motion and persist them."""
        if self.calibration and self.sts_calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, "
                "or type 'c' and press ENTER to run full calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                with self.bus.torque_disabled():
                    self.bus.write_calibration(self.calibration)
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
            self.calibration[motor] = MotorCalibration(
                id=int(m.SlaveID),
                motor_offset=float(homing_offsets[motor]),
                range_min=float(range_mins[motor]),
                range_max=float(range_maxes[motor]),
            )

        with self.bus.torque_disabled():
            self.bus.write_calibration(self.calibration)

        print("Move STS joints through their ranges of motion. Press ENTER to stop...")
        sts_range_mins, sts_range_maxes = self.sts_bus.record_ranges_of_motion()
        self.sts_calibration = {}
        for motor in self.sts_bus.motors:
            self.sts_calibration[motor] = StsMotorCalibration(
                id=self.sts_bus.motors[motor].id,
                offset=0.0,
                range_min=float(sts_range_mins[motor]),
                range_max=float(sts_range_maxes[motor]),
            )
        self.sts_bus.write_calibration(self.sts_calibration, cache=True)
        self._sts_has_calibration = True
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        """Apply post-connection tweaks (no-op for now)."""
        pass

    def get_observation(self) -> dict[str, Any]:
        """Return the latest motor state and camera frames."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        start = time.perf_counter()
        motor_state = self.bus.sync_read()
        obs_dict = motor_state.copy()
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        start = time.perf_counter()
        sts_state = self.sts_bus.sync_read(normalize=self._sts_has_calibration)
        obs_dict.update(sts_state)
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read sts state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command the arm to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Raises:
            RobotDeviceNotConnectedError: if robot is not connected.

        Returns:
            dict[str, Any]: Mapping ``<motor>.<pos|vel|tor>`` describing the actual command
            that was sent after clipping.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        time_start = time.perf_counter()

        goal_positions = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}
        damiao_goal_positions = {
            motor: val for motor, val in goal_positions.items() if motor in self.bus.motor_names
        }
        sts_goal_positions = {
            motor: val for motor, val in goal_positions.items() if motor in self.sts_bus.motor_names
        }

        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        if self.config.max_relative_target is not None and damiao_goal_positions:
            present_pos = self.bus.sync_read()
            goal_present_pos = {
                motor: (g_pos, present_pos[f"{motor}.pos"]) for motor, g_pos in damiao_goal_positions.items()
            }
            damiao_goal_positions = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        command = action.copy()
        for motor, val in damiao_goal_positions.items():
            command[f"{motor}.pos"] = val

        # Send goal position to the arm
        self.bus.sync_write(command)
        if sts_goal_positions:
            sts_command = {f"{motor}.pos": command[f"{motor}.pos"] for motor in self.sts_bus.motor_names}
            self.sts_bus.sync_write(sts_command, normalize=self._sts_has_calibration)
        dt_ms = (time.perf_counter() - time_start) * 1e3
        logger.debug(f"{self} sent action: {dt_ms:.1f}ms")
        return command

    def disconnect(self):
        """Gracefully disconnect the robot and attached cameras."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        if self.sts_bus.is_connected:
            self.sts_bus.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
