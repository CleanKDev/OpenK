# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

"""
Simple script to control a robot from teleoperation.

Example:

```shell
lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/tty.usbmodem58760431541 \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 1920, height: 1080, fps: 30}}" \
    --robot.id=black \
    --teleop.type=so101_leader \
    --teleop.port=/dev/tty.usbmodem58760431551 \
    --teleop.id=blue \
    --display_data=true
```

Example teleoperation with bimanual so100:

```shell
lerobot-teleoperate \
  --robot.type=bi_so100_follower \
  --robot.left_arm_port=/dev/tty.usbmodem5A460851411 \
  --robot.right_arm_port=/dev/tty.usbmodem5A460812391 \
  --robot.id=bimanual_follower \
  --robot.cameras='{
    left: {"type": "opencv", "index_or_path": 0, "width": 1920, "height": 1080, "fps": 30},
    top: {"type": "opencv", "index_or_path": 1, "width": 1920, "height": 1080, "fps": 30},
    right: {"type": "opencv", "index_or_path": 2, "width": 1920, "height": 1080, "fps": 30}
  }' \
  --teleop.type=bi_so100_leader \
  --teleop.left_arm_port=/dev/tty.usbmodem5A460828611 \
  --teleop.right_arm_port=/dev/tty.usbmodem5A460826981 \
  --teleop.id=bimanual_leader \
  --display_data=true
```

"""

import logging
import time
import sys
import os
from dataclasses import asdict, dataclass
from pprint import pformat
import importlib
from pathlib import Path

# Add project root to sys.path to allow importing 'utils' and 'openk'
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rerun as rr

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401
from lerobot.configs import parser
from lerobot.processor import (
    RobotAction,
    RobotObservation,
    RobotProcessorPipeline,
    make_default_processors,
)
from lerobot.robots import (  # noqa: F401
    Robot,
    RobotConfig,
    make_robot_from_config,
)
from lerobot.teleoperators import (  # noqa: F401
    Teleoperator,
    TeleoperatorConfig,
    make_teleoperator_from_config,
)
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep as busy_wait
from lerobot.utils.utils import init_logging, move_cursor_up
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
from utils.log_utils import rename_daily_log_file_name
from utils.motor_state_table import format_motor_state_table
from utils.direction_map import DirectionMapProcessorStep
from utils.motion_utils import build_home_action, move_home_and_disable_torque, ramp_action

def _register_openk():
    # デコレータによる型登録を発火させる
    for m in (
        "openk.openk-1-alpha_follower",
        "openk.openk-1-alpha_leader",
        "openk.openk-1-alpha_follower2",
        "openk.openk-1-sts_leader",
        "openk.one-motor-test_follower",
        "openk.one-motor-test_leader",
    ):
        importlib.import_module(m)


@dataclass
class TeleoperateConfig:
    # TODO: pepijn, steven: if more robots require multiple teleoperators (like lekiwi) its good to make this possibele in teleop.py and record.py with List[Teleoperator]
    teleop: TeleoperatorConfig
    robot: RobotConfig
    # Optional sign flips per joint (motor base name -> +/-1.0)
    direction_map: dict[str, float] | None = None
    # Dataset config (ignored by teleoperation but allowed for config reuse)
    dataset: dict | None = None
    # Limit the maximum frames per second.
    fps: int = 60
    teleop_time_s: float | None = None
    # Display all cameras on screen
    display_data: bool = False
    # Display action/observation values in the CLI
    display_cli: bool = False
    # Force calibration dialog on connect
    always_calibrate: bool = False
    # Optional log file path for init_logging
    log_file: str | Path | None = "logs/openk_teleoperate.log"
    # If true, appends the date to the log_file name
    daily_log_file_name: bool = False
    # Ramp to first teleop action to avoid sudden motion
    start_ramp_time_s: float = 2.0
    start_ramp_dt_s: float = 0.05
    # Return home and disable torque on exit
    end_home_ramp_time_s: float = 3.0
    end_home_ramp_dt_s: float = 0.05
    end_home_settle_s: float = 1.0

def teleop_loop(
    teleop: Teleoperator,
    robot: Robot,
    fps: int,
    teleop_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_action_processor: RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction],
    robot_observation_processor: RobotProcessorPipeline[RobotObservation, RobotObservation],
    display_data: bool = False,
    display_cli: bool = False,
    duration: float | None = None,
    *,
    start_ramp_time_s: float = 0.0,
    start_ramp_dt_s: float = 0.05,
):
    """
    This function continuously reads actions from a teleoperation device, processes them through optional
    pipelines, sends them to a robot, and optionally displays the robot's state. The loop runs at a
    specified frequency until a set duration is reached or it is manually interrupted.

    Args:
        teleop: The teleoperator device instance providing control actions.
        robot: The robot instance being controlled.
        fps: The target frequency for the control loop in frames per second.
        display_data: If True, fetches robot observations and displays them in Rerun.
        display_cli: If True, prints action/observation values to the CLI.
        duration: The maximum duration of the teleoperation loop in seconds. If None, the loop runs indefinitely.
        teleop_action_processor: An optional pipeline to process raw actions from the teleoperator.
        robot_action_processor: An optional pipeline to process actions before they are sent to the robot.
        robot_observation_processor: An optional pipeline to process raw observations from the robot.
    """

    start = time.perf_counter()

    ramp_done = False
    while True:
        loop_start = time.perf_counter()

        # Get robot observation
        # Not really needed for now other than for visualization
        # teleop_action_processor can take None as an observation
        # given that it is the identity processor as default
        obs = robot.get_observation()

        # Get teleop action
        raw_action = teleop.get_action()

        # Process teleop action through pipeline
        teleop_action = teleop_action_processor((raw_action, obs))

        # Process action for robot through pipeline
        robot_action_to_send = robot_action_processor((teleop_action, obs))

        if not ramp_done and start_ramp_time_s > 0:
            ramp_action(
                robot,
                obs,
                robot_action_to_send,
                duration_s=start_ramp_time_s,
                dt_s=start_ramp_dt_s,
            )
            ramp_done = True
        else:
            # Send processed action to robot (robot_action_processor.to_output should return dict[str, Any])
            _ = robot.send_action(robot_action_to_send)

        if display_data or display_cli:
            # Process robot observation through pipeline
            obs_transition = robot_observation_processor(obs)

        if display_data:
            log_rerun_data(
                observation=obs_transition,
                action=teleop_action,
            )

        if display_cli:
            table, line_count = format_motor_state_table(
                robot_action_to_send,
                obs_transition,
            )
            print(table)
            move_cursor_up(line_count + 2)

        dt_s = time.perf_counter() - loop_start
        busy_wait(1 / fps - dt_s)
        loop_s = time.perf_counter() - loop_start
        print(f"\ntime: {loop_s * 1e3:.2f}ms ({1 / loop_s:.0f} Hz)")

        if duration is not None and time.perf_counter() - start >= duration:
            return


@parser.wrap()
def teleoperate(cfg: TeleoperateConfig):
    # Create parent dir if a default log file path is used to avoid FileNotFoundError.
    if cfg.log_file is not None:

        if cfg.daily_log_file_name:
            cfg.log_file = rename_daily_log_file_name(cfg.log_file)

        Path(cfg.log_file).parent.mkdir(parents=True, exist_ok=True)
    init_logging(log_file=cfg.log_file)
    logging.info(pformat(asdict(cfg)))
    if cfg.display_data:
        init_rerun(session_name="teleoperation")

    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    if cfg.direction_map:
        teleop_action_processor.steps.append(DirectionMapProcessorStep(cfg.direction_map))

    teleop.connect(calibrate=cfg.always_calibrate)
    robot.connect(calibrate=cfg.always_calibrate)

    try:
        teleop_loop(
            teleop=teleop,
            robot=robot,
            fps=cfg.fps,
            display_data=cfg.display_data,
            display_cli=cfg.display_cli,
            duration=cfg.teleop_time_s,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            start_ramp_time_s=cfg.start_ramp_time_s,
            start_ramp_dt_s=cfg.start_ramp_dt_s,
        )
    except KeyboardInterrupt:
        logging.info("Teleoperation interrupted by user.")
        pass
    finally:
        if cfg.display_data:
            rr.rerun_shutdown()
        teleop.disconnect()
        #logging.info("Teleoperator disconnected.")
        try:
            home_action = build_home_action(robot.action_features)
            move_home_and_disable_torque(
                robot,
                home_action=home_action,
                ramp_time_s=cfg.end_home_ramp_time_s,
                ramp_dt_s=cfg.end_home_ramp_dt_s,
                settle_time_s=cfg.end_home_settle_s,
            )
        except Exception:
            logging.exception("Failed to return robot home on exit.")
        robot.disconnect()
        #logging.info("Robot disconnected.")



def main():
    register_third_party_plugins()
    _register_openk()
    teleoperate()


if __name__ == "__main__":
    main()
