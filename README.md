**日本語** | [English](docs/README_en.md) | [中文](docs/README_zh.md)

# OpenK on lerobot

オープンソース低コストAIロボットアームOpenK。10万円台で作ることが可能です。
板金不要。ボディ部分はすべて3Dプリンタで作成可能。URDF完備
DAMIAOモーターを使用し、lerobotコードベースで動作します。
ローカル `.venv` と `uv` で開発・実行します。
トイレ清掃ロボット[CleanK](https://cleank.space)を開発する過程で生まれた汎用アームです。

# DEMO
## テレオペレーション
<https://github.com/user-attachments/assets/53bd0b76-32d7-4085-9f98-901ec6ca804d>
## データセットビジュアライザー
lerobotのデータセット形式に準拠しているためSO-ARM101と同じく可視化が可能
<https://huggingface.co/spaces/lerobot/visualize_dataset?path=%2FHodaka0706%2Fcleank_test_dataset_cam%2Fepisode_0>


https://github.com/user-attachments/assets/60f25825-fb9d-405b-99c9-d5dd31f4e1e4




# CAD

## Follower

- <https://github.com/CleanKDev/OpenK/blob/main/hardware/follower/OpenKSTEP.step>
<img width="355" height="646" alt="image" src="https://github.com/user-attachments/assets/b3e61dc8-b2bb-4c2e-8feb-bde9bd71bdcf" />

## Leader

- <https://github.com/CleanKDev/OpenK/blob/main/hardware/leader/small-leader.step>
<img width="387" height="395" alt="image" src="https://github.com/user-attachments/assets/e6cb0c62-0938-41ab-9ce5-0a8835269c25" />

# Actuators

アクチュエーター構成：
・肩×2：DM-J6006
・上腕・肘・前腕：DM-J4310
・手首・グリッパー：STS3215
・リーダーアーム：全軸STS3215

# Core Members

- [@sabamiso-rrsc](https://x.com/sabamiso_RRSC) ほぼすべてのハードウェア、組み込み開発、DAMIAOイニシャライズ　ソフトウェア
- [@UedaKenji](https://x.com/kenji5012) lerobotコード統合。メインソフトウェア開発
- [@shunyatadano](https://x.com/tdshun) URDF及びシミュレーション
- @shinshin0706, [@Ryosuke520](https://x.com/Ryosuke_Nkgw), [@deBroglieeeen](https://x.com/_nm_inp) 現場レベルのインテグレーション及びハードウェア、ソフトウェアの調整、シミュレーション開発

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

- LeRobot by HuggingFace, Inc.
- SO-100 by TheRobotStudio
- 洛星ロボット研究部の皆様　<https://robot.rakusei.net/>
- OpenArm by Enactic, inc.
- ロボ⭐︎スタディオン及び会員の皆様
- DMM.make TIB FAB
