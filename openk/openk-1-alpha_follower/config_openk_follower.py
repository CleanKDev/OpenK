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

from dataclasses import dataclass, field
from pathlib import Path

from lerobot.cameras import CameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.robots import RobotConfig
from ..damiao.damiao import MotorNormMode
from ..damiao.DM_CAN import Control_Type
from ..sts import StsNormMode


@RobotConfig.register_subclass("openk-1-alpha_follower")
@dataclass(kw_only=True)
class OpenKFollowerConfig(RobotConfig):
    """Static configuration for the OpenK follower arm (ports, safety bounds, cameras)."""
    # Port to connect to the arm
    port: str
    sts_port: str

    disable_torque_on_disconnect: bool = False
    control_type: Control_Type = Control_Type.POS_VEL
    motor_norm_mode: MotorNormMode = MotorNormMode.RANGE_M100_100
    sts_motor_norm_mode: StsNormMode = StsNormMode.RANGE_M100_100

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a dictionary that maps motor
    # names to the max_relative_target value for that motor.
    max_relative_target: float | dict[str, float] | None = None
    calibration_dir: Path | None = None

    # cameras
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "cam_1": RealSenseCameraConfig(
                serial_number_or_name="850312071789",
                fps=30,
                width=320,
                height=240,
                use_depth=False,
            )
        }
    )
