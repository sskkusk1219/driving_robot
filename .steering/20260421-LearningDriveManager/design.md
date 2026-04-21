# 設計書

## アーキテクチャ概要

既存のレイヤードアーキテクチャに従い、`src/domain/learning_drive.py` に実装する（ドメインレイヤー）。
ハードウェア依存は Protocol 注入で隠蔽し、ユニットテストはモックで完結させる。

```
ドメインレイヤー
  LearningDriveManager
    ├── generate_patterns(profile) → list[LearningPattern]  ← 純粋関数
    ├── run_pattern(pattern) → LearningLog                  ← async, HW Protocol注入
    └── train_model(logs, profile_id) → str                 ← ファイル保存
```

## コンポーネント設計

### 1. LearningPattern（データクラス / models/learning_drive.py）

**責務**: 1回のパターン走行の指令値を保持する。

```python
@dataclass
class LearningPattern:
    speed_kmh: float       # グリッド上の目標速度 [km/h]
    accel_kmhs: float      # グリッド上の目標加速度 [km/h/s]（負=減速）
    accel_opening: float   # 指令アクセル開度 [%]
    brake_opening: float   # 指令ブレーキ開度 [%]
    hold_duration_s: float # 開度保持時間 [s]
```

### 2. LearningLog（データクラス / models/learning_drive.py）

**責務**: run_pattern の実行結果を保持する。

```python
@dataclass
class LearningLog:
    pattern: LearningPattern
    actual_speed_kmh: float        # 保持期間中の平均実車速 [km/h]
    accel_opening_applied: float   # 実際に適用したアクセル開度 [%]
    brake_opening_applied: float   # 実際に適用したブレーキ開度 [%]
    recorded_at: datetime
```

### 3. LearningDriveManager（src/domain/learning_drive.py）

**責務**:
- パターングリッドの生成とフィルタリング
- パターンの実行とログ収集
- 収集ログからの pkl モデル構築と保存

**実装の要点**:
- `generate_patterns` は純粋関数（HW 依存なし）
- `run_pattern` は Protocol 注入（`LearningActuatorProtocol`, `LearningCANProtocol`）
- `train_model` は numpy + pickle のみ（scipy 不使用、グリッドが疎なため）

### 4. Protocol 定義（src/domain/learning_drive.py 内）

```python
class LearningActuatorProtocol(Protocol):
    async def move_to_position(self, pos: int) -> None: ...
    async def read_position(self) -> int: ...

class LearningCANProtocol(Protocol):
    async def read_speed(self) -> float: ...
```

## データフロー

### パターン生成
```
VehicleProfile
  → 速度グリッド: 0〜max_speed を SPEED_STEP_KMH 刻み
  → 加速度グリッド: -max_decel_kmhs〜ACCEL_MAX_KMHS を ACCEL_STEP_KMHS 刻み
  → 各点の初期開度を線形マッピングで計算
  → max_opening / max_decel_g 超過パターンを除外
  → list[LearningPattern]
```

### パターン走行
```
LearningPattern
  → accel/brake 開度 → pulse 換算（CalibrationData 不要、% を直値で使用）
  → move_to_position(accel_pulse), move_to_position(brake_pulse)
  → asyncio.sleep(hold_duration_s)
  → read_speed() でサンプリング → 平均
  → LearningLog
```

**注意**: 位置換算は `opening% / 100 * max_stroke_pulse` とするが、
CalibrationData が必要なため、`run_pattern` には `CalibrationData` も引数に取る。

### モデル学習
```
list[LearningLog]
  → パターンの (speed_kmh, accel_kmhs) でグループ化
  → numpy array に変換: speed_grid, accel_grid, accel_map[N,M], brake_map[N,M]
  → scipy.interpolate.griddata で疎データを正規グリッドに補間
  → pickle で data/models/{profile_id}_{timestamp}.pkl に保存
  → pkl パスを返す
```

## エラーハンドリング戦略

### カスタムエラークラス
```python
class LearningDataError(Exception):
    """ログが不足・不正でモデル構築できない場合に送出。"""
```

### エラーハンドリングパターン
- `train_model`: ログが 4 点未満（補間不可）→ `LearningDataError`
- `train_model`: 速度グリッドが 1 点しかない → `LearningDataError`
- `generate_patterns`: 全パターンがフィルタで除外 → 空リストを返す（例外なし）

## テスト戦略

### ユニットテスト（tests/unit/test_learning_drive.py）
- `generate_patterns`: グリッド生成、フィルタリング、境界値
- `train_model`: pkl 構造の検証、FeedforwardController で読み込み確認、エラーケース
- `run_pattern`: モックアクチュエータ・CAN で LearningLog が正しく生成されること

## 依存ライブラリ

既存の依存関係のみ使用（追加なし）:
- `numpy` — グリッド配列・pkl データ
- `scipy` — `griddata` による疎データ補間
- `pickle` — pkl 保存（標準ライブラリ）

## ディレクトリ構造

```
src/
  models/
    learning_drive.py          # LearningPattern, LearningLog dataclass（新規）
  domain/
    learning_drive.py          # LearningDriveManager（新規）
tests/
  unit/
    test_learning_drive.py     # ユニットテスト（新規）
data/
  models/                      # pkl 保存先（既存ディレクトリ）
```

## 実装の順序

1. `src/models/learning_drive.py` — LearningPattern / LearningLog dataclass
2. `src/domain/learning_drive.py` — LearningDriveManager 本体
3. `tests/unit/test_learning_drive.py` — ユニットテスト

## セキュリティ考慮事項

- pkl 保存先を `data/models/` に固定し、パストラバーサルを防ぐ（profile_id にスラッシュを含む場合は `Path(profile_id).name` で安全化）

## パフォーマンス考慮事項

- `generate_patterns` はグリッドサイズが最大でも 10×10=100 点程度のため計算コストは無視できる
- `train_model` の scipy.griddata は 100 点程度なら数 ms 以内
