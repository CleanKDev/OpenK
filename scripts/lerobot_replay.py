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
Replays the actions of an episode from a dataset on a robot.

Example (OpenK):
```shell
lerobot-replay \
  --robot.type=openk-1-alpha_follower \
  --robot.port=/dev/ttyUSB0 \
  --robot.id=openk_follower \
  --dataset.repo_id=<my_username>/<my_dataset_name> \
  --dataset.episode=0
```

"""

import logging
import time
import sys
import os
from dataclasses import asdict, dataclass
import importlib
from pathlib import Path
from pprint import pformat

# Add project root to sys.path for local module imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lerobot.configs import parser
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.processor import (
    make_default_robot_action_processor,
)
from lerobot.robots import RobotConfig, make_robot_from_config
from lerobot.utils.constants import ACTION
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep as busy_wait
from lerobot.utils.utils import (
    init_logging,
    log_say,
)


def _register_openk():
    # Ensure OpenK follower is registered.
    for module in (
        "openk.openk-1-alpha_follower",
        "openk.openk-1-alpha_follower2",
        "openk.openk-1-alpha_leader",
    ):
        importlib.import_module(module)


@dataclass
class DatasetReplayConfig:
    # Dataset identifier. By convention it should match '{hf_username}/{dataset_name}' (e.g. `lerobot/test`).
    repo_id: str
    # Episode to replay.
    episode: int
    # Root directory where the dataset will be stored (e.g. 'dataset/path').
    root: str | Path | None = None
    # Limit the frames per second. By default, uses the policy fps.
    fps: int = 30


@dataclass
class ReplayConfig:
    robot: RobotConfig
    dataset: DatasetReplayConfig
    # Use vocal synthesis to read events.
    play_sounds: bool = True


@parser.wrap()
def replay(cfg: ReplayConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    robot_action_processor = make_default_robot_action_processor()

    robot = make_robot_from_config(cfg.robot)
    dataset = LeRobotDataset(cfg.dataset.repo_id, root=cfg.dataset.root, episodes=[cfg.dataset.episode])

    # Filter dataset to only include frames from the specified episode since episodes are chunked in dataset V3.0
    episode_frames = dataset.hf_dataset.filter(lambda x: x["episode_index"] == cfg.dataset.episode)
    actions = episode_frames.select_columns(ACTION)

    robot.connect()

    log_say("Replaying episode", cfg.play_sounds, blocking=True)
    for idx in range(len(episode_frames)):
        start_episode_t = time.perf_counter()

        action_array = actions[idx][ACTION]
        action = {}
        for i, name in enumerate(dataset.features[ACTION]["names"]):
            action[name] = action_array[i]

        robot_obs = robot.get_observation()

        processed_action = robot_action_processor((action, robot_obs))

        _ = robot.send_action(processed_action)

        dt_s = time.perf_counter() - start_episode_t
        busy_wait(1 / dataset.fps - dt_s)

    robot.disconnect()


def main():
    register_third_party_plugins()
    _register_openk()
    replay()


if __name__ == "__main__":
    main()
