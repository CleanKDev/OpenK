[日本語](../README.md) | **English** | [中文](README_zh.md)

# OpenK on lerobot

An open-source, low-cost AI robot arm. Can be built for around $1,000.

No sheet metal required — all body parts are fully 3D-printable. URDF included.
Uses DAMIAO motors and runs on the lerobot codebase.
Development and execution use a local `.venv` with `uv`.

Originally developed as a general-purpose arm during the creation of [CleanK](https://cleank.space), a public restroom cleaning robot.

# DEMO

<https://github.com/user-attachments/assets/53bd0b76-32d7-4085-9f98-901ec6ca804d>

# CAD

## Follower

- <https://github.com/CleanKDev/OpenK/blob/main/hardware/follower/OpenKSTEP.step>
<img width="355" height="646" alt="image" src="https://github.com/user-attachments/assets/b3e61dc8-b2bb-4c2e-8feb-bde9bd71bdcf" />

## Leader

- <https://github.com/CleanKDev/OpenK/blob/main/hardware/leader/small-leader.step>
<img width="387" height="395" alt="image" src="https://github.com/user-attachments/assets/e6cb0c62-0938-41ab-9ce5-0a8835269c25" />

# Actuators

Actuator configuration:

- Shoulder x2: DM-J6006
- Upper arm / Elbow / Forearm: DM-J4310
- Wrist / Gripper: STS3215
- Leader arm: STS3215 for all axes

# Core Members

- [@sabamiso-rrsc](https://x.com/sabamiso_RRSC) — Most hardware, embedded development, DAMIAO initialization software
- [@UedaKenji](https://x.com/kenji5012) — lerobot code integration, main software development
- [@shunyatadano](https://x.com/tdshun) — URDF and simulation
- @shinshin0706, [@Ryosuke520](https://x.com/Ryosuke_Nkgw), [@deBroglieeeen](https://x.com/_nm_inp) — Field-level integration, hardware/software tuning, simulation development

## Setup

- Prerequisites: Python 3.10 / [uv](https://github.com/astral-sh/uv)
- First time:

  ```bash
  git clone <repo-url> lerobot_OpenK
  cd lerobot_OpenK
  uv venv .venv --python 3.10
  source .venv/bin/activate
  uv pip install -e .
  ```

- For subsequent sessions, just run `source .venv/bin/activate`.

## Testing

- Standard (no hardware required):

  ```bash
  UV_CACHE_DIR=.uv_cache uv run pytest
  ```

- Hardware-in-the-loop (HIL) tests (optional):

  ```bash
  OPENK_HIL=1 \
  OPENK_FOLLOWER_PORT=/dev/ttyUSB0 \
  OPENK_LEADER_PORT=/dev/ttyUSB1 \
  UV_CACHE_DIR=.uv_cache uv run pytest test/test_openk_hil.py -q
  ```

  *Camera is disabled; only connection + single observation/action retrieval is tested.*

## Key Scripts

- If you don't know your port, run port discovery first:

  ```bash
  uv run lerobot-find-port
  ```

  Check for `ttyUSB*` etc., then run the commands below.
- Teleoperation:

  ```bash
  uv run openk-teleoperate \
    --robot.type=openk-1-alpha_follower --robot.port=<follower-port> --robot.id=follower \
    --teleop.type=openk_leader --teleop.port=<leader-port> --teleop.id=leader \
    --log_file=logs/openk_teleoperate.log
  ```

  - Press Ctrl+C to stop
- Recording:

  ```bash
  uv run openk-record \
    --robot.type=openk-1-alpha_follower --robot.port=<follower-port> --robot.id=follower \
    --teleop.type=openk_leader --teleop.port=<leader-port> --teleop.id=leader \
    --dataset.repo_id=<user>/<dataset> --dataset.num_episodes=2 --dataset.single_task="Describe task" \
    --log_file=logs/openk_record.log
  ```

- Replay:

  ```bash
  uv run lerobot-replay \
    --robot.type=openk-1-alpha_follower --robot.port=<follower-port> --robot.id=follower \
    --dataset.repo_id=<user>/<dataset> --dataset.episode=0
  ```

- Calibration:

  ```bash
  uv run lerobot-calibrate --teleop.type=openk_leader --teleop.port=<leader-port> --teleop.id=leader
  ```

- Other utilities: `lerobot-find-port`, `lerobot-find-cameras`, `lerobot-info`.

### Running from Config Files

- Example YAML files are included:
  - `configs/teleop_example.yaml`
  - `configs/record_example.yaml`
- Usage:

  ```bash
  uv run openk-teleoperate --config_path=config/teleop_example.yaml
  uv run openk-record --config_path=config/record_example.yaml
  ```

- Keys specified in the YAML are applied as-is; unspecified keys use defaults (to keep `cameras`, simply omit the key).

## Logs

- Teleoperation/recording logs are written to `logs/openk_teleoperate.log` / `logs/openk_record.log` by default (parent directories are created automatically). Override with `--log_file`.

## Notes

- The `lerobot` dependency in `pyproject.toml` uses an editable reference to `../lerobot`. Adjust the path if your setup differs.

# Acknowledgments

- LeRobot by HuggingFace, Inc.
- SO-100 by TheRobotStudio
- Rakusei Robotics Club — <https://robot.rakusei.net/>
- OpenArm by Enactic, Inc.
- Robo Stadion and its members
- DMM.make TIB FAB
