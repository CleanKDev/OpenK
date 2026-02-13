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

from pathlib import Path
from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.motors import MotorNormMode as FeetechMotorNormMode
from lerobot.robots import RobotConfig

from ..damiao.DM_CAN import Control_Type
from ..damiao.damiao import MotorNormMode as DamiaoMotorNormMode


@dataclass(frozen=True)
class DamiaoMotorSpec:
    name: str
    motor_type: str
    slave_id: int
    master_id: int


@dataclass(frozen=True)
class StsMotorSpec:
    name: str
    motor_id: int
    model: str = "sts3215"


@RobotConfig.register_subclass("openk-1-alpha_follower2")
@dataclass(kw_only=True)
class OpenKFollower2Config(RobotConfig):
    """Static configuration for the OpenK follower arm with Feetech STS gripper."""

    # Ports: Damiao bus and Feetech STS bus
    port: str
    sts_port: str

    calibration_dir: Path | None = None
    calibration_root: Path | None = None

    disable_torque_on_disconnect: bool = False

    control_type: Control_Type = Control_Type.MIT
    motor_norm_mode: DamiaoMotorNormMode = DamiaoMotorNormMode.RANGE_M100_100
    # Motor definitions used to build the Damiao/STS buses.
    damiao_motors: list[DamiaoMotorSpec] = field(
        default_factory=lambda: [
            DamiaoMotorSpec("shoulder_pan", "DM6006", 0x01, 0x15),
            DamiaoMotorSpec("shoulder_lift", "DM6006", 0x02, 0x15),
            DamiaoMotorSpec("shoulder_roll", "DM4310", 0x03, 0x15),
            DamiaoMotorSpec("elbow_flex", "DM4310", 0x04, 0x15),
            DamiaoMotorSpec("wrist_roll", "DM4310", 0x05, 0x15),
        ]
    )

    # Optional per-motor gains/limits (float for global, dict for per-motor).
    damiao_kp: float | dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": 20.0,
            "shoulder_lift": 20.0,
            "shoulder_roll": 20.0,
            "elbow_flex": 20.0,
            "wrist_roll": 20.0,
        }
    )
    damiao_kd: float | dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": 1.0,
            "shoulder_lift": 1.0,
            "shoulder_roll": 1.0,
            "elbow_flex": 1.0,
            "wrist_roll": 1.0,
        }
    )
    # These params only apply in Position Control mode.
    damiao_velocity_limit: float | dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": 2.0,
            "shoulder_lift": 2.0,
            "shoulder_roll": 2.0,
            "elbow_flex": 2.0,
            "wrist_roll": 2.0,
        }
    )
    sts_motor_norm_mode: FeetechMotorNormMode = FeetechMotorNormMode.RANGE_M100_100
    sts_motors: list[StsMotorSpec] = field(
        default_factory=lambda: [
            StsMotorSpec("wrist_flex", 2),
            StsMotorSpec("grip", 1),
        ]
    )

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a dictionary that maps motor
    # names to the max_relative_target value for that motor.
    max_relative_target: float | dict[str, float] | None = None

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
