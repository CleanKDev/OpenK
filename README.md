# OpenK

## What This Repository Is / このリポジトリについて / 关于本仓库

- JP: OpenK は、低コストなオープンソースロボットアームを `lerobot` ベースで動かすための実装です。ハードウェア（CAD/STL/URDF）と、テレオペ・記録・再生・キャリブレーション用スクリプトをまとめています。
- EN: OpenK is a low-cost open-source robot arm project built on top of `lerobot`. This repo contains hardware assets (CAD/STL/URDF) and software scripts for teleoperation, data recording, replay, and calibration.
- 中文: OpenK 是一个基于 `lerobot` 的低成本开源机械臂项目。仓库包含硬件资源（CAD/STL/URDF）以及用于遥操作、数据采集、回放和标定的软件脚本。

## Demo

- https://github.com/user-attachments/assets/53bd0b76-32d7-4085-9f98-901ec6ca804d

## Repository Layout / 構成 / 仓库结构

- `hardware/follower/OpenKSTEP.step`: follower CAD (STEP)
- `hardware/leader/small-leader.step`: leader CAD (STEP)
- `hardware/follower/robot.urdf`: follower URDF
- `hardware/follower/*.stl`, `hardware/leader/*.stl`: mesh files
- `scripts/`: operation scripts (teleop/record/calibrate/replay/tools)
- `config/`: example config files for teleop/record/replay
- `openk/`: robot and teleoperator implementations
- `test/`: tests

## Quick Start / クイックスタート / 快速开始

### 1) Prerequisites / 前提 / 前置条件

- JP: Python `3.10+`、`uv`、シリアル接続可能な環境（USB/COM）。
- EN: Python `3.10+`, `uv`, and a serial-capable environment (USB/COM).
- 中文: 需要 Python `3.10+`、`uv`，以及可用串口环境（USB/COM）。

### 2) Setup / セットアップ / 安装

```bash
git clone <repo-url> OpenK
cd OpenK
uv venv .venv --python 3.10
```

- Linux/macOS:

```bash
source .venv/bin/activate
```

- Windows PowerShell:

```powershell
.venv\Scripts\Activate.ps1
```

```bash
uv pip install -e .
```

### 3) Find Ports / ポート確認 / 串口确认

```bash
uv run python scripts/lerobot_find_port.py
```

- JP: follower/leader のポートを確認します（例: `/dev/ttyUSB0`, `/dev/ttyUSB1` または `COM3`, `COM4`）。
- EN: Identify follower/leader serial ports (e.g., `/dev/ttyUSB0`, `/dev/ttyUSB1` or `COM3`, `COM4`).
- 中文: 确认 follower/leader 串口（例如 `/dev/ttyUSB0`、`/dev/ttyUSB1` 或 `COM3`、`COM4`）。

### 4) Calibrate / キャリブレーション / 标定

```bash
uv run python scripts/lerobot_calibrate.py --config_path=config/calibrate_openk_sts_leader.yaml
```

### 5) Teleoperate / テレオペ実行 / 运行遥操作

```bash
uv run python scripts/cleank_teleoperate.py --config_path=config/teleop_example.yaml
```

- JP: 終了は `Ctrl + C`。
- EN: Stop with `Ctrl + C`.
- 中文: 使用 `Ctrl + C` 结束。

## Data Recording / データ記録 / 数据采集

```bash
uv run python scripts/cleank_record.py --config_path=config/record_example.yaml
```

- JP: 収集データは設定に応じて保存されます。必要に応じて `dataset.repo_id` などを `config/*.yaml` で変更してください。
- EN: Data is stored according to config. Update `dataset.repo_id` and related fields in `config/*.yaml` as needed.
- 中文: 数据按配置写入。请按需在 `config/*.yaml` 中修改 `dataset.repo_id` 等参数。

## Replay / リプレイ / 回放

```bash
uv run python scripts/lerobot_replay.py --config_path=config/replay_follower2.yaml
```

## Testing / テスト / 测试

### Standard tests / 通常テスト / 常规测试

```bash
UV_CACHE_DIR=.uv_cache uv run pytest
```

### HIL tests (optional) / 実機接続テスト(任意) / 硬件在环测试（可选）

```bash
OPENK_HIL=1 \
OPENK_FOLLOWER_PORT=/dev/ttyUSB0 \
OPENK_LEADER_PORT=/dev/ttyUSB1 \
UV_CACHE_DIR=.uv_cache uv run pytest test/test_cleank_hil.py -q
```

## Typical Workflow / 典型フロー / 典型流程

- JP: `ポート確認 -> キャリブレーション -> テレオペ -> 記録 -> リプレイ`
- EN: `find ports -> calibrate -> teleoperate -> record -> replay`
- 中文: `确认串口 -> 标定 -> 遥操作 -> 采集 -> 回放`

## Troubleshooting / トラブルシュート / 故障排查

- JP: ポート接続に失敗する場合は、まず `lerobot_find_port.py` で再確認し、設定ファイル内のポート名を合わせてください。
- EN: If connection fails, re-check ports with `lerobot_find_port.py` and update config port values.
- 中文: 若连接失败，请先用 `lerobot_find_port.py` 重新确认串口，并更新配置文件中的端口。
- JP: 依存関係の解決に失敗する場合は、仮想環境を作り直して `uv pip install -e .` を再実行してください。
- EN: If dependency resolution fails, recreate the virtual environment and rerun `uv pip install -e .`.
- 中文: 若依赖安装失败，请重建虚拟环境并重新执行 `uv pip install -e .`。

## Credits

- LeRobot by HuggingFace, Inc.
- SO-100 by TheRobotStudio
