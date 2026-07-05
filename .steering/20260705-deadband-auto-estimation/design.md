# 設計書

## アーキテクチャ概要

既存の学習運転パターン生成(`src/domain/learning_drive.py`)・状態機械(`src/domain/control/learning_loop.py`)・物理定数推定(`src/domain/model_training.py`)の3レイヤーに変更を加える。**学習運転の状態機械(`learning_loop.py`)自体には一切変更を加えない** — 既存の`_Phase.MEASURE`汎用ロジック(パターンの`accel_opening`/`brake_opening`を無ランプで即時適用し`hold_duration_s`経過で次へ進む)と、既存の`BRAKE_HOLD`パターン種別をそのまま再利用することで実現する。

```
学習運転パターン列生成(learning_drive.py)
  └ CREEP → CREEP_SETTLE → [新規: ACCEL_DEADBAND_PROBE ×5段] → ACCEL_SWEEP ×4段
    → BRAKE_HOLD ×8段(低開度4段を追加) → COAST_DOWN ×3本
       ↓ (状態機械は無変更。ACCEL_DEADBAND_PROBE は既存 _Phase.MEASURE 経路へ自動的に乗る)
学習ループ実行(learning_loop.py, 無変更) → drive_logs（連続ログ）
       ↓
物理定数推定(model_training.py: estimate_dynamics_params)
  └ 新規: 開度→応答曲線のビン分割・オンセット検出で accel_deadband_pct / brake_deadband_pct を推定
       ↓
training_service.train_and_apply → profile.feedforward_params 更新 → DB永続化・制御スタック反映
```

## 設計判断

### 判断1: 新パターン種別はMEASURE系の直接値保持(ランプなし)にする

`PatternKind.ACCEL_DEADBAND_PROBE`は、既存の`_initial_phase()`のロジック上、`ACCEL_SWEEP`/`BRAKE_HOLD`/`COAST_DOWN`のいずれにも該当しないため自動的に`_Phase.MEASURE`へ入る。MEASURE フェーズは`pattern.accel_opening`/`pattern.brake_opening`を無ランプで即座に適用し、`pattern.hold_duration_s`経過で`_advance_pattern`する既存の汎用ロジックであり、`CREEP`パターン(ブレーキを段階的に無ランプで緩める)と全く同じ形。開度が0.5〜5%と極めて小さいため、無ランプでの直接印加は`CREEP`の前例と同様に機械的ジャークの懸念がない。**`learning_loop.py`のコード変更は一切不要**。

### 判断2: ブレーキ不感帯プローブは新パターン種別を追加せず、`BRAKE_HOLD_OPENINGS_PCT`に低い値を追加するだけ

既存の`BRAKE_HOLD`は「アクセルで加速→固定ブレーキ開度を保持して定常減速を記録」という、まさに欲しい形(低開度を保持して応答を見る)そのものである。低いブレーキ開度(1/2/3/5%)を追加するだけで、既存の状態機械・タイムアウト処理をそのまま使い回せる。低開度では減速が緩いため`brake_hold_timeout_s`(既定20秒)で打ち切られる可能性が高いが、それでも部分データとして問題なく使える(既存の他開度でも同じタイムアウト処理を共有)。

### 判断3: プローブ配置はCREEP_SETTLE直後・ACCEL_SWEEP開始前

クリープ安定待ち(CREEP_SETTLE)は「アクセル・ブレーキ0%での定常応答」を確立する区間であり、これが不感帯推定のベースライン(開度ゼロ近傍の基準応答)と物理的に同じ状態である。この直後に低開度を昇順(0.5→5%)で保持することで、ベースラインからの連続的な开度上昇に対する応答変化を自然に追跡できる。各プローブ間で明示的な停車リセットは行わない(開度差分が小さく、蓄積速度も小さいため、続くACCEL_SWEEPのDRIVE_ACCELフェーズが残存速度に関わらず正しく動作する)。

### 判断4: 推定アルゴリズムはビン分割+オンセット検出(既存FOPDT同定のスタイルを踏襲)

開度を`DEADBAND_BIN_WIDTH_PCT`(0.5%)幅でビン分割し、各ビンの中央値応答(アクセルは`dv`、ブレーキは`-dv`)を計算する。開度ゼロ近傍ビン(bin 0)の中央値を「無反応時ベースライン」とし、それを`DEADBAND_ONSET_MARGIN_KMHS`(0.3 km/h/s、既存の`CREEP_STEADY_TOL_KMHS`と同スケール)以上上回る最初のビンの下端開度を不感帯境界とする。各ビンは`DEADBAND_MIN_BIN_SAMPLES`(5、既存`MIN_OBS_SAMPLES`と同値)以上のサンプルを要求し、ベースラインが求まらない・境界が見つからない場合は`None`を返し、呼び出し元が既存値を保持する(`_median_or_none`と同じ堅牢性方針)。

### 判断5: 推定値のクランプ

`DEADBAND_SCAN_MAX_PCT`(10.0%)を推定の探索上限とし、これを超える不感帯は物理的に非現実的として検出しない(既存値を保持)。境界が見つかった場合の返り値は自動的に`[0, DEADBAND_SCAN_MAX_PCT]`の範囲内になる。

## コンポーネント設計

### 1. `src/models/learning_drive.py`

**追加**: `PatternKind.ACCEL_DEADBAND_PROBE = "accel_deadband_probe"`

### 2. `src/domain/learning_drive.py`

**追加定数**:
```python
ACCEL_DEADBAND_PROBE_PCTS: tuple[float, ...] = (0.5, 1.0, 2.0, 3.0, 5.0)
ACCEL_DEADBAND_PROBE_HOLD_S: float = 3.0
```

**変更定数**: `BRAKE_HOLD_OPENINGS_PCT`に低開度4段を追加
```python
BRAKE_HOLD_OPENINGS_PCT: tuple[float, ...] = (1.0, 2.0, 3.0, 5.0, 10.0, 20.0, 30.0, 40.0)
```

**`LearningDriveConfig`に追加**: `accel_deadband_probe_pcts`, `accel_deadband_probe_hold_s`

**`generate_patterns`**: CREEP_SETTLE追加直後、ACCEL_SWEEPループ開始前に以下を挿入:
```python
for pct in self._config.accel_deadband_probe_pcts:
    accel = min(max(pct, 0.0), profile.max_accel_opening)
    if accel <= 0.0:
        continue
    patterns.append(
        LearningPattern(
            kind=PatternKind.ACCEL_DEADBAND_PROBE,
            accel_opening=accel,
            brake_opening=0.0,
            hold_duration_s=self._config.accel_deadband_probe_hold_s,
        )
    )
```

### 3. `src/domain/control/learning_loop.py`

**変更なし。** `_initial_phase`/`_command_openings`/`_advance`の既存MEASURE分岐がそのまま`ACCEL_DEADBAND_PROBE`を処理する。

### 4. `src/domain/model_training.py`

**追加定数**:
```python
DEADBAND_BIN_WIDTH_PCT: float = 0.5
DEADBAND_SCAN_MAX_PCT: float = 10.0
DEADBAND_ONSET_MARGIN_KMHS: float = 0.3
DEADBAND_MIN_BIN_SAMPLES: int = 5
```

**追加関数** `_estimate_onset_deadband_pct(openings: np.ndarray, response: np.ndarray) -> float | None`:
開度ビン分割→ベースライン(bin 0)算出→マージンを超える最初のビンの下端を返す。上記「判断4」参照。

**`estimate_dynamics_params`の変更**:
- ループ内で`accel_scan_openings`/`accel_scan_dv`(アクセル用: ブレーキオフ・開度<=SCAN_MAX)、`brake_scan_openings`/`brake_scan_decel`(ブレーキ用: アクセルオフ・開度<=SCAN_MAX、応答は`-dv`)をセッション横断で収集
- ループ後に`_estimate_onset_deadband_pct`を各々に適用し、結果があれば`accel_deadband_pct`/`brake_deadband_pct`を上書き、なければ`current`を保持
- 冒頭のdocstringから「不感帯は推定対象外」の記述を削除し、新ロジックを説明する記述に更新

## データフロー

```
1. 学習運転: CREEP → CREEP_SETTLE(基準応答確立) → ACCEL_DEADBAND_PROBE×5(昇順, 各3秒無ランプ保持)
   → ACCEL_SWEEP×4 → BRAKE_HOLD×8(低開度4段含む) → COAST_DOWN×3
2. 全区間が同一の連続ログ(drive_logs)として記録される(パターン種別はログに残らない。
   推定はopening/dvの生値のみから統計的に行う)
3. TRAINING_1/2: estimate_dynamics_params が上記ログから accel/brake deadband を推定
4. プロファイルへ永続化・制御スタック(PedalArbiter)へ即時反映
```

## テスト戦略

### ユニットテスト
- `tests/unit/test_learning_drive.py`: `generate_patterns`が`ACCEL_DEADBAND_PROBE`パターンを CREEP_SETTLE 直後・ACCEL_SWEEP 前に含むこと(順序・開度昇順・hold_duration)。`BRAKE_HOLD_OPENINGS_PCT`拡張後も低開度パターンが生成されること。既存テストの回帰確認。
- `tests/unit/test_model_training.py`: `_estimate_onset_deadband_pct`の単体テスト(明確なオンセットがあるケース、オンセットが見つからないケース、サンプル不足でNoneになるケース)。`estimate_dynamics_params`が十分なプローブ風データがあるとき不感帯を上書きし、不足時は既存値を保持すること。

### 統合テスト
- 既存の`train_and_apply`関連テスト(`tests/unit/test_training_service.py`)は`estimate_dynamics_params`をモックしているため無変更で通ることを確認。

## リスクと対処

| リスク | 対処 |
|---|---|
| プローブ開度でも僅かに車両が動き加速度ノイズで誤検出 | ビンごとの最小サンプル数閾値とマージンで統計的信頼性を担保。閾値未満は既存値保持 |
| 旧ログ(プローブ導入前)のみでの再学習で不感帯が更新されない | 意図通り(サンプル不足→既存値保持、後方互換) |
| ACCEL_DEADBAND_PROBE後に残存速度がACCEL_SWEEPに影響 | 開度・保持時間が小さく蓄積速度は僅少。DRIVE_ACCELは残存速度に関わらず正しく動作するため実害なし |
| BRAKE_HOLD低開度追加でタイムアウト(20秒)に頻繁に達し学習時間が伸びる | 既存の他開度でも同じタイムアウトを共有する既存動作であり、既知の許容コスト |

## 実装の順序

1. `src/models/learning_drive.py`: PatternKind追加 + テスト
2. `src/domain/learning_drive.py`: 定数追加・generate_patterns変更 + テスト
3. `src/domain/model_training.py`: 推定ロジック追加 + テスト
4. 全体回帰テスト・lint・型チェック
