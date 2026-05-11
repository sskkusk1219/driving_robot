# 設計書

## アーキテクチャ概要

ハードウェアテストスクリプト `tests/hardware/test_calibration.py` を新規作成する。
`CalibrationManager`（ドメインクラス）は変更せず、`ActuatorDriver` を直接使用する独立スクリプト。

```
【スクリプト構造】

test_calibration.py
    ├── _read_state(driver)           → (current_ma, position_pulse)
    ├── _probe_contact(target, other, target_is_accel, start_pos)
    │       └── CalibrationManager._probe_contact() と同一アルゴリズム
    │           + 両軸の電流・位置を毎ステップ出力
    ├── _calibrate_one_axis(target, other, target_is_accel, label)
    │       └── ゼロ検出 → フル検出 → home_return
    └── main()
            ├── 接続・初期化（PMSL → reset_alarm → servo_on → home_return）
            ├── アクセル軸キャリブレーション
            ├── ブレーキ軸キャリブレーション
            └── 結果表示
```

## コンポーネント設計

### 定数

```python
_MOVE_STEP_PULSE = 50          # 1ステップの移動量 [pulse]
_STEP_INTERVAL_S = 0.05        # ステップ間隔 [s]（CalibrationConfig デフォルト値と同一）
_CURRENT_WINDOW = 5            # 電流移動平均ウィンドウ幅（CalibrationConfig と同一）
_CURRENT_SPIKE_RATIO = 1.5     # スパイク判定倍率（CalibrationConfig と同一）
_OVERCURRENT_LIMIT_MA = 300.0  # 安全閾値（スパイク判定とは別）
_MAX_SEARCH_PULSE = 50000      # 最大探索距離 [pulse]
_MOVE_SPEED_MM_S = 5           # キャリブレーション中の移動速度
```

### スパイク判定アルゴリズム（CalibrationManager._probe_contact() と同一）

```python
window: deque[float]（maxlen=5）
baseline: float（最初のウィンドウ平均 = 自由移動中の基準電流）
if current > avg + baseline * 1.5:
    → 接触点または終端を検出
```

### 出力フォーマット

移動中（毎ステップ）:
```
  電流 a={a_cur:7.1f}mA b={b_cur:7.1f}mA  位置 a={a_pos:6d} b={b_pos:6d}  (閾値:{_OVERCURRENT_LIMIT_MA:.0f}mA)
```

ゼロフル検出後（最終結果）:
```
acc : {accel_zero * 0.01:.2f} mm = 0%, {accel_full * 0.01:.2f} mm = 100%  brk : {brake_zero * 0.01:.2f} mm = 0%, {brake_full * 0.01:.2f} mm = 100%
```

## データフロー

### 片軸キャリブレーション

```
1. home_return()（安全開始位置）
2. ゼロ点検出: _probe_contact(start_pos=0)
   - STEP ずつ正方向に移動
   - 毎ステップ: 両軸の電流・位置を asyncio.gather で並列読み取り → 1行出力
   - スパイク検出 → read_position() → zero_pos
3. move_to_position(zero_pos)（接触点に移動して待機）
4. フル点検出: _probe_contact(start_pos=zero_pos)
   - 同様の処理
   - スパイク検出 → full_pos
5. home_return()（安全位置に戻す）
```

### 実行順序

```
アクセル軸キャリブレーション
  ゼロ点検出中: a=動いている, b=静止中（接触点の検出フロー）
  フル点検出中: a=動いている, b=静止中

ブレーキ軸キャリブレーション
  ゼロ点検出中: a=静止中, b=動いている
  フル点検出中: a=静止中, b=動いている
```

## 実装ファイル

- **新規**: `tests/hardware/test_calibration.py`
- **変更なし**: `src/domain/calibration.py`（既存ドメインクラス）
- **変更なし**: その他すべての既存ファイル

## エラーハンドリング

| エラー | 対処 |
|--------|------|
| Modbus 接続失敗 | エラー表示して即終了 |
| read_current/read_position 失敗 | (0.0, 0) を返してループ継続 |
| 接触点未検出（最大探索超過） | RuntimeError を raise → main でキャッチして原点復帰 |
| KeyboardInterrupt | home_return 両軸 → close |
