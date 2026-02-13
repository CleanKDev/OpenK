# lerobot-openk

OpenK ロボット向けの LeRobot 拡張。ローカル `.venv` と `uv` で開発・実行します。

# DEMO
https://github.com/user-attachments/assets/2fc6349e-fb53-4c70-8322-91da34f47839

# CAD
## Follower
https://github.com/CleanKDev/OpenK/blob/main/hardware/follower/OpenKSTEP.step

## Leader
https://github.com/CleanKDev/OpenK/blob/main/hardware/leader/small-leader.step

# Core Members
@sabamiso-rrsc ほぼすべてのハードウェア、組み込み開発、DAMIAOイニシャライズ　ソフトウェア
@UedaKenji lerobotコード統合。メインソフトウェア開発
@shunyatadano URDF及びシミュレーション
@shinshin0706, Ryosuke520, deBroglieeeen 現場レベルのインテグレーション及びハードウェア、ソフトウェアの調整、シミュレーション開発 
## セットアップ
- 前提: Python 3.10 / [uv](https://github.com/astral-sh/uv)
- 初回:
  ```bash
  git clone <repo-url> lerobot_OpenK
  cd lerobot_OpenK
  uv venv .venv --python 3.10
  source .venv/bin/activate
  uv pip install -e .
  ```
- 以降の作業時は `source .venv/bin/activate` のみで OK。

## テスト
- 通常（ハードウェア不要）:
  ```bash
  UV_CACHE_DIR=.uv_cache uv run pytest
  ```
- 実機接続の HIL テスト（オプション）:
  ```bash
  OPENK_HIL=1 \
  OPENK_FOLLOWER_PORT=/dev/ttyUSB0 \
  OPENK_LEADER_PORT=/dev/ttyUSB1 \
  UV_CACHE_DIR=.uv_cache uv run pytest test/test_openk_hil.py -q
  ```
  ※カメラは無効化し、接続→1回観測/アクション取得のみ。

## 主要スクリプト
- ポートが不明な場合は先にポート探索:  
  ```bash
  uv run lerobot-find-port
  ```
  で `ttyUSB*` などを確認してから下記を実行。
- テレオペ:  
  ```bash
  uv run openk-teleoperate \
    --robot.type=openk-1-alpha_follower --robot.port=<follower-port> --robot.id=follower \
    --teleop.type=openk_leader --teleop.port=<leader-port> --teleop.id=leader \
    --log_file=logs/openk_teleoperate.log
  ```
  - Ctrl + C で停止

- 記録:  
  ```bash
  uv run openk-record \
    --robot.type=openk-1-alpha_follower --robot.port=<follower-port> --robot.id=follower \
    --teleop.type=openk_leader --teleop.port=<leader-port> --teleop.id=leader \
    --dataset.repo_id=<user>/<dataset> --dataset.num_episodes=2 --dataset.single_task="Describe task" \
    --log_file=logs/openk_record.log
  ```
- リプレイ:  
  ```bash
  uv run lerobot-replay \
    --robot.type=openk-1-alpha_follower --robot.port=<follower-port> --robot.id=follower \
    --dataset.repo_id=<user>/<dataset> --dataset.episode=0
  ```
- キャリブレーション:  
  ```bash
  uv run lerobot-calibrate --teleop.type=openk_leader --teleop.port=<leader-port> --teleop.id=leader
  ```
- その他ユーティリティ: `lerobot-find-port`, `lerobot-find-cameras`, `lerobot-info`。

### 設定ファイルから実行する場合
- 例の YAML を同梱:
  - `configs/teleop_example.yaml`
  - `configs/record_example.yaml`
- 実行例:
  ```bash
  uv run openk-teleoperate --config_path=config/teleop_example.yaml
  uv run openk-record --config_path=config/record_example.yaml
  ```
- YAML に書いたキーはそのまま適用、書かないキーはデフォルトのまま（`cameras` を残したい場合はキー自体を書かない）。

## ログ
- テレオペ/記録はデフォルトで `logs/openk_teleoperate.log` / `logs/openk_record.log` に出力（親ディレクトリは自動生成）。`--log_file` で上書き可能。

## 補足
- `pyproject.toml` の `lerobot` 依存はローカル `../lerobot` を editable 参照しています。パスが異なる場合は適宜修正してください。
-

# 謝辞
LeRobot by HuggingFace, Inc.
SO-100 by TheRobotStudio
