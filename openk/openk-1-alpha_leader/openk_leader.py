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
from typing import Any
from pathlib import Path
import draccus

from lerobot.teleoperators import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, TELEOPERATORS

from ..damiao.DM_CAN import DM_Motor_Type, Motor
from ..damiao.damiao import DamiaoMotorsBus, MotorCalibration
from .config_openk_leader import OpenkLeaderConfig


logger = logging.getLogger(__name__)


class OpenkLeader(Teleoperator):
    """Teleoperator wrapper around the Damiao-powered OpenK leader arm."""

    config_class = OpenkLeaderConfig
    name = "openk_leader"

    def __init__(self, config: OpenkLeaderConfig):
        super().__init__(config)
        self.config = config
        # super().__init__(config)の代わりの初期化を行う
        self.id = config.id
        self.calibration_dir = (
            HF_LEROBOT_CALIBRATION / TELEOPERATORS / self.name
            if self.config.calibration_dir is None
            else Path(self.config.calibration_dir)
        )
        self.calibration_dir.mkdir(parents=True, exist_ok=True)
        self.calibration_fpath = self.calibration_dir / f"{self.id}.json"
        self.calibration: dict[str, MotorCalibration] = {}
        if self.calibration_fpath.is_file():
            self._load_calibration()

        #######################################################

        self.config = config
        self.bus = DamiaoMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan": Motor(DM_Motor_Type.DM6006, 0x01, 0x15),
                "shoulder_lift": Motor(DM_Motor_Type.DM6006, 0x02, 0x15),
                "shoulder_roll": Motor(DM_Motor_Type.DM4310, 0x03, 0x15),
                "elbow_flex": Motor(DM_Motor_Type.DM4310, 0x04, 0x15),
                "wrist_roll": Motor(DM_Motor_Type.DM4310, 0x05, 0x15),
            },
            calibration=self.calibration,
            motor_norm_mode=self.config.motor_norm_mode,
            control_type=self.config.control_type,
        )

    @property
    def action_features(self) -> dict[str, type]:
        """The leader streams normalized joint positions and velocities for each motor."""
        features = {}
        for motor in self.bus.motor_names:
            features[f"{motor}.pos"] = float
            features[f"{motor}.vel"] = float
        return features

    @property
    def feedback_features(self) -> dict[str, type]:
        """No force feedback is implemented for this leader."""
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        """Open the motor bus and optionally run calibration."""
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()

        requires_calibration = calibrate  # フラグが True なら絶対に対話モードへ

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

    def _load_calibration(self, fpath: Path | None = None) -> None:
        """
        Helper to load calibration data from the specified file.

        Args:
            fpath (Path | None): Optional path to the calibration file. Defaults to `self.calibration_fpath`.
        """
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath) as f, draccus.config_type("json"):
            self.calibration = draccus.load(dict[str, MotorCalibration], f)


    def calibrate(self) -> None:
        """Interactively record offsets/ranges of motion and persist them."""
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
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


        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        """Hook reserved for post-connection adjustments."""
        pass

    def get_action(self) -> dict[str, Any]:
        """Return the latest normalized joint positions and velocities from the leader arm."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        motor_state = self.bus.sync_read()
        action = {
            key: val
            for key, val in motor_state.items()
            if key.endswith(".pos") or key.endswith(".vel")
        }
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """No haptic/force feedback is supported yet."""
        raise NotImplementedError("Force feedback is not implemented for the OpenK leader.")

    def disconnect(self) -> None:
        """Disconnect the teleoperator safely."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        logger.info(f"{self} disconnected.")
