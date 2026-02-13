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

from lerobot.motors import MotorNormMode
from lerobot.teleoperators import TeleoperatorConfig


@dataclass(frozen=True)
class StsMotorSpec:
    name: str
    motor_id: int
    model: str = "sts3215"


@TeleoperatorConfig.register_subclass("openk-1-sts_leader")
@dataclass(kw_only=True)
class OpenkStsLeaderConfig(TeleoperatorConfig):
    """Static configuration for the OpenK STS leader teleoperator."""

    port: str
    calibration_dir: Path | None = None
    calibration_root: Path | None = None
    disable_torque_on_disconnect: bool = True
    motor_norm_mode: MotorNormMode = MotorNormMode.RANGE_M100_100
    sts_motors: list[StsMotorSpec] = field(
        default_factory=lambda: [
            StsMotorSpec("shoulder_pan", 1),
            StsMotorSpec("shoulder_lift", 2),
            StsMotorSpec("shoulder_roll", 3),
            StsMotorSpec("elbow_flex", 4),
            StsMotorSpec("wrist_roll", 5),
            StsMotorSpec("wrist_flex", 6),
            StsMotorSpec("grip", 7),
        ]
    )
