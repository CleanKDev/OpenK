[日本語](../README.md) | [English](README_en.md) | **中文**

# OpenK on lerobot

开源低成本AI机械臂OpenK。总成本约7000元人民币即可制作。
本项目是在开发厕所清洁机器人[CleanK](https://cleank.space)的过程中诞生的通用机械臂。
无需钣金加工，机身部分全部可通过3D打印完成。附带完整URDF。
使用DAMIAO电机，基于lerobot代码库运行。
使用本地 `.venv` 和 `uv` 进行开发与运行。

# 演示
https://github.com/user-attachments/assets/53bd0b76-32d7-4085-9f98-901ec6ca804d

# CAD
## Follower（从臂）
- https://github.com/CleanKDev/OpenK/blob/main/hardware/follower/OpenKSTEP.step
<img width="355" height="646" alt="image" src="https://github.com/user-attachments/assets/b3e61dc8-b2bb-4c2e-8feb-bde9bd71bdcf" />


## Leader（主臂）
- https://github.com/CleanKDev/OpenK/blob/main/hardware/leader/small-leader.step
<img width="387" height="395" alt="image" src="https://github.com/user-attachments/assets/e6cb0c62-0938-41ab-9ce5-0a8835269c25" />

# 执行器
执行器配置：
- 肩部 x2：DM-J6006
- 上臂 / 肘部 / 前臂：DM-J4310
- 腕部 / 夹爪：STS3215
- 主臂：全轴STS3215

# 核心成员
- [@sabamiso-rrsc](https://x.com/sabamiso_RRSC) — 大部分硬件、嵌入式开发、DAMIAO初始化软件
- [@UedaKenji](https://x.com/kenji5012) — lerobot代码集成，主要软件开发
- [@shunyatadano](https://x.com/tdshun) — URDF及仿真
- @shinshin0706、[@Ryosuke520](https://x.com/Ryosuke_Nkgw)、[@deBroglieeeen](https://x.com/_nm_inp) — 现场级集成、软硬件调试、仿真开发

## 环境搭建
- 前提条件：Python 3.10 / [uv](https://github.com/astral-sh/uv)
- 首次安装：
  ```bash
  git clone <repo-url> lerobot_OpenK
  cd lerobot_OpenK
  uv venv .venv --python 3.10
  source .venv/bin/activate
  uv pip install -e .
  ```
- 之后每次使用只需执行 `source .venv/bin/activate` 即可。

## 测试
- 常规测试（无需硬件）：
  ```bash
  UV_CACHE_DIR=.uv_cache uv run pytest
  ```
- 硬件在环（HIL）测试（可选）：
  ```bash
  OPENK_HIL=1 \
  OPENK_FOLLOWER_PORT=/dev/ttyUSB0 \
  OPENK_LEADER_PORT=/dev/ttyUSB1 \
  UV_CACHE_DIR=.uv_cache uv run pytest test/test_openk_hil.py -q
  ```
  ※摄像头已禁用，仅测试连接及单次观测/动作获取。

## 主要脚本
- 如果不确定端口号，先运行端口探索：
  ```bash
  uv run lerobot-find-port
  ```
  确认 `ttyUSB*` 等信息后，再执行以下命令。
- 遥操作：
  ```bash
  uv run openk-teleoperate \
    --robot.type=openk-1-alpha_follower --robot.port=<follower-port> --robot.id=follower \
    --teleop.type=openk_leader --teleop.port=<leader-port> --teleop.id=leader \
    --log_file=logs/openk_teleoperate.log
  ```
  - 按 Ctrl+C 停止
- 记录：
  ```bash
  uv run openk-record \
    --robot.type=openk-1-alpha_follower --robot.port=<follower-port> --robot.id=follower \
    --teleop.type=openk_leader --teleop.port=<leader-port> --teleop.id=leader \
    --dataset.repo_id=<user>/<dataset> --dataset.num_episodes=2 --dataset.single_task="Describe task" \
    --log_file=logs/openk_record.log
  ```
- 回放：
  ```bash
  uv run lerobot-replay \
    --robot.type=openk-1-alpha_follower --robot.port=<follower-port> --robot.id=follower \
    --dataset.repo_id=<user>/<dataset> --dataset.episode=0
  ```
- 标定：
  ```bash
  uv run lerobot-calibrate --teleop.type=openk_leader --teleop.port=<leader-port> --teleop.id=leader
  ```
- 其他工具：`lerobot-find-port`、`lerobot-find-cameras`、`lerobot-info`。

### 使用配置文件运行
- 附带示例YAML文件：
  - `configs/teleop_example.yaml`
  - `configs/record_example.yaml`
- 运行示例：
  ```bash
  uv run openk-teleoperate --config_path=config/teleop_example.yaml
  uv run openk-record --config_path=config/record_example.yaml
  ```
- YAML中指定的键直接生效，未指定的键保持默认值（如需保留 `cameras`，不写该键即可）。

## 日志
- 遥操作/记录默认输出到 `logs/openk_teleoperate.log` / `logs/openk_record.log`（父目录自动创建）。可通过 `--log_file` 覆盖。

## 补充说明
- `pyproject.toml` 中的 `lerobot` 依赖以可编辑模式引用 `../lerobot`。如路径不同请自行修改。

# 致谢
- LeRobot by HuggingFace, Inc.
- SO-100 by TheRobotStudio
- 洛星机器人研究部 — https://robot.rakusei.net/
- OpenArm by Enactic, Inc.
- Robo Stadium及全体会员
- DMM.make TIB FAB
