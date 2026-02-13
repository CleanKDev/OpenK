# LeRobot Hugging Face Dataset ガイド (OpenK)

このノートは、`scripts/` と `config/` の内容に基づいて、OpenK で
LeRobot データセットを Hugging Face Hub に登録・更新・読み取りする方法をまとめたものです。

参照元:
- `scripts/openk_record.py`
- `scripts/lerobot_replay.py`
- `config/record_example.yaml`

## 1. 登録 (Hub にデータセットを作る)

データセットは記録時に作成されます。重要なのは `dataset.repo_id` で、
`<hf_username>/<dataset_name>` の形式にします。

例 (README より):

```bash
uv run openk-record \
  --robot.type=openk-1-alpha_follower --robot.port=<follower-port> --robot.id=follower \
  --teleop.type=openk_leader --teleop.port=<leader-port> --teleop.id=leader \
  --dataset.repo_id=<user>/<dataset> --dataset.num_episodes=2 --dataset.single_task="Describe task" \
  --log_file=logs/openk_record.log
```

主なオプション (`scripts/openk_record.py` の `DatasetRecordConfig`):
- `--dataset.repo_id`: Hub のリポジトリ ID。必須。
- `--dataset.single_task`: 収録タスクの説明。各フレームの `task` として保存。
- `--dataset.push_to_hub`: デフォルト `true`。記録終了後にアップロード。
- `--dataset.private`: `true` で非公開。
- `--dataset.tags`: 付与するタグのリスト。

注意: Hub へのアップロードには認証が必要です
(`huggingface-cli login` や `HF_TOKEN` など)。

## 2. ローカル保存先

`--dataset.root` を指定しない場合は HF datasets のデフォルトキャッシュ
(`~/.cache/huggingface/datasets/`) に保存されます
(`config/record_example.yaml` のコメント参照)。

例:
```bash
--dataset.root=./data
```

## 3. 既存データセットの更新 (追記)

同じ `repo_id` で `--resume=true` を付けると、既存のデータセットにエピソードを追記します。
`root` を指定した場合は同じパスを使います。

`scripts/openk_record.py` の流れ:
- `resume=true` の場合 `LeRobotDataset(...)` を作成
- `dataset.add_frame(...)` でフレーム保存
- `dataset.save_episode()` でエピソード確定
- `push_to_hub=true` なら `dataset.push_to_hub(...)`

例:
```bash
uv run openk-record \
  --config_path=config/record_example.yaml \
  --dataset.repo_id=<user>/<dataset> \
  --resume=true
```

ヒント: まずローカルで記録したい場合は `--dataset.push_to_hub=false` にして、
後で `dataset.push_to_hub(...)` を呼ぶこともできます。

## 4. データ構造 (何が保存されるか)

データのスキーマは動的に構成され、以下が結合されます:
- ロボット観測 (observation) の特徴量
- テレオペ or ポリシーのアクション特徴量
- パイプラインによる追加特徴

`scripts/openk_record.py` では `dataset.features` が構成され、
各フレームに以下が含まれます:
- `task`: 文字列 (`single_task`)
- 観測フィールド (接頭辞 `OBS_STR`)
- アクションフィールド (接頭辞 `ACTION`)
- `episode_index` などのエピソード情報

ロボット・カメラ・ポリシー設定により列名が変わるため、
`dataset.features` を実際に確認してから解析するのが安全です。

`--dataset.video=true` の場合は `VideoEncodingManager` により動画化され、
`false` の場合は PNG 画像として保存されます。

## 5. 読み取りと解析

### A. LeRobotDataset で読み込む (このリポジトリの標準)
```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ds = LeRobotDataset("<user>/<dataset>", root=None)
print("fps:", ds.fps)
print("num_episodes:", ds.num_episodes)
print("features:", ds.features)

hf = ds.hf_dataset
print("columns:", hf.column_names)

# 例: あるエピソードだけ抽出
ep0 = hf.filter(lambda x: x["episode_index"] == 0)
actions = ep0.select_columns("action")
```

### B. Replay の流れに沿ったアクション抽出
(`scripts/lerobot_replay.py` 相当)
```python
episode_frames = ds.hf_dataset.filter(lambda x: x["episode_index"] == 0)
actions = episode_frames.select_columns("action")
for i in range(len(episode_frames)):
    action_vec = actions[i]["action"]
    # action 名は ds.features["action"]["names"] を参照
```

### C. pandas で簡易集計
```python
df = ds.hf_dataset.to_pandas()
counts = df.groupby("episode_index").size()
print(counts)
```

※ `to_pandas()` はデータ量が多いと重くなる点に注意してください。

## 6. 実運用上の注意点

- FPS は一致必須: `record_loop` 内で `dataset.fps == cfg.dataset.fps` を確認。
- スキーマは設定依存。列名を固定せず `features` を見てから扱う。
- 观測/アクション名を変更する場合は `--dataset.rename_map` を使い互換性確保。
- 動画保存時は `--dataset.video_encoding_batch_size` と
  image writer のスレッド数を調整して FPS を安定化。

## 7. 関連ファイル

- 記録・アップロード: `scripts/openk_record.py`
- リプレイ (エピソード抽出): `scripts/lerobot_replay.py`
- 設定例: `config/record_example.yaml`, `config/record_one_motor_test.yaml`
