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
from functools import cached_property
from pathlib import Path
from typing import Any

import draccus

from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, TELEOPERATORS
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.robots import Robot
from lerobot.robots.utils import ensure_safe_goal_position

from ..damiao.DM_CAN import DM_Motor_Type, Motor
from ..damiao.damiao import DamiaoMotorsBus, MotorCalibration
from .config_follower import OneMotorFollowerConfig

logger = logging.getLogger(__name__)


class OneMotorFollower(Robot):
    """Robot wrapper around a single Damiao motor for teleop tests."""

    config_class = OneMotorFollowerConfig
    name = "one_motor_test_follower"

    def __init__(self, config: OneMotorFollowerConfig):
        super().__init__(config)

        self.id = config.id
        self.calibration_dir = (
            Path(config.calibration_dir)
            if config.calibration_dir is not None
            else HF_LEROBOT_CALIBRATION / TELEOPERATORS / self.name
        )
        self.calibration_dir.mkdir(parents=True, exist_ok=True)
        self.calibration_fpath = self.calibration_dir / f"{self.id}.json"
        self.calibration: dict[str, MotorCalibration] = {}
        if self.calibration_fpath.is_file():
            self._load_calibration()

        self.config = config
        self.bus = DamiaoMotorsBus(
            port=self.config.port,
            motors={"motor1": Motor(DM_Motor_Type.DM4310, 0x01, 0x15)},
            calibration=self.calibration,
            motor_norm_mode=config.motor_norm_mode,
            control_type=config.control_type,
        )

    @cached_property
    def observation_features(self) -> dict[str, type]:
        res: dict[str, type] = {}
        res.update({f"{motor}.pos": float for motor in self.bus.motor_names})
        res.update({f"{motor}.vel": float for motor in self.bus.motor_names})
        res.update({f"{motor}.tor": float for motor in self.bus.motor_names})
        return res

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motor_names}

    def _load_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath) as f, draccus.config_type("json"):
            self.calibration = draccus.load(dict[str, MotorCalibration], f)

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        """Open the motor bus and optionally run calibration."""
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()

        requires_calibration = calibrate

        if not requires_calibration:
            if not self.calibration:
                logger.info("キャリブレーションファイルなし，あるいはキーの不一致。")
                requires_calibration = True
            elif not self.bus.check_offset():
                logger.warning("モータのオフセットとファイルが不一致。")
                requires_calibration = True
            else:
                logger.info("キャッシュ済みキャリブレーションがそのまま使えます。")

        if requires_calibration:
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        """Interactively record offsets/ranges of motion and persist them."""
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motor")
                with self.bus.torque_disabled():
                    self.bus.write_calibration(self.calibration)
                return

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
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        """Apply post-connection tweaks (no-op for now)."""
        pass

    def get_observation(self) -> dict[str, Any]:
        """Return the latest motor state."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        motor_state = self.bus.sync_read()
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")
        return motor_state

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command the arm to move to a target joint configuration."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        time_start = time.perf_counter()
        goal_positions = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read()
            goal_present_pos = {
                motor: (g_pos, present_pos[f"{motor}.pos"]) for motor, g_pos in goal_positions.items()
            }
            goal_positions = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        command = action.copy()
        for motor, val in goal_positions.items():
            command[f"{motor}.pos"] = val

        self.bus.sync_write(command)
        dt_ms = (time.perf_counter() - time_start) * 1e3
        logger.debug(f"{self} sent action: {dt_ms:.1f}ms")
        return command

    def disconnect(self):
        """Gracefully disconnect the robot."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        logger.info(f"{self} disconnected.")
