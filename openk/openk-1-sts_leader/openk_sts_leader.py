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

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.teleoperators import Teleoperator
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS, TELEOPERATORS
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_openk_sts_leader import OpenkStsLeaderConfig, StsMotorSpec

logger = logging.getLogger(__name__)


def _get_spec_value(spec: object, key: str):
    if hasattr(spec, key):
        return getattr(spec, key)
    if isinstance(spec, dict) and key in spec:
        return spec[key]
    raise KeyError(f"Missing '{key}' in motor spec: {spec}")


def _build_sts_motors(
    specs: list[StsMotorSpec | dict[str, object]],
    norm_mode: MotorNormMode,
) -> dict[str, Motor]:
    motors: dict[str, Motor] = {}
    for spec in specs:
        name = str(_get_spec_value(spec, "name"))
        motor_id = int(_get_spec_value(spec, "motor_id"))
        model = str(_get_spec_value(spec, "model"))
        motors[name] = Motor(motor_id, model, norm_mode)
    return motors


class OpenkStsLeader(Teleoperator):
    """STS-based leader teleoperator for the OpenK follower arm."""

    config_class = OpenkStsLeaderConfig
    name = "openk_sts_leader"

    def __init__(self, config: OpenkStsLeaderConfig):
        super().__init__(config)
        self.config = config
        self.id = config.id
        if self.config.calibration_dir is not None:
            self.calibration_dir = Path(self.config.calibration_dir)
        elif self.config.calibration_root is not None:
            self.calibration_dir = Path(self.config.calibration_root) / TELEOPERATORS / self.name
        else:
            self.calibration_dir = HF_LEROBOT_CALIBRATION / TELEOPERATORS / self.name
        self.calibration_dir.mkdir(parents=True, exist_ok=True)
        self.calibration_fpath = self.calibration_dir / f"{self.id}.json"
        self.calibration: dict[str, MotorCalibration] = {}
        if self.calibration_fpath.is_file():
            self._load_calibration()

        motors = _build_sts_motors(self.config.sts_motors, self.config.motor_norm_mode)
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors=motors,
            calibration=self.calibration,
        )

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def _load_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath) as f, draccus.config_type("json"):
            self.calibration = draccus.load(dict[str, MotorCalibration], f)

    def _save_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath, "w") as f, draccus.config_type("json"):
            draccus.dump(self.calibration, f, indent=2)

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect(handshake=False)
        if not self.is_calibrated or calibrate:
            if not self.is_calibrated:
                logger.info("No calibration found or calibration file mismatch.")
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.bus.write_calibration(self.calibration)
                return
        else:
            print(f"Calibration file missing for {self.id} at {self.calibration_fpath}")
            user_input = input("Would you like to run calibration now? [y/N]: ")
            if user_input.strip().lower() != "y":
                print("Exiting as calibration is required to continue.")
                exit(0)

        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()

        print(
            "Move all joints sequentially through their entire ranges "
            "of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion()

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        try:
            self.bus.disable_torque()
            self.bus.configure_motors()
            for motor in self.bus.motors:
                self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                self.bus.write("P_Coefficient", motor, 16)
                self.bus.write("I_Coefficient", motor, 0)
                self.bus.write("D_Coefficient", motor, 32)
            self.bus.disable_torque()
        except Exception as e:
            logger.warning(f"Error during leader STS configuration: {e}. Proceeding anyway...")

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        obs_dict = self.bus.sync_read("Present_Position", num_retry=3)
        action = {f"{motor}.pos": val for motor, val in obs_dict.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        raise NotImplementedError("Force feedback is not implemented for the OpenK STS leader.")

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        logger.info(f"{self} disconnected.")
