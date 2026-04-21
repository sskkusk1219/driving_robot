# CalibrationManager 設計

## アーキテクチャ上の位置づけ
- ドメインレイヤー（`src/domain/calibration.py`）
- アクチュエータドライバは `CalibrationActuatorProtocol` で抽象化
- 既存の `robot_controller.py` が Protocol パターンで HW を隠蔽しているのと同じ方式

## クラス構成

### 追加: `src/models/calibration.py`
```python
@dataclass
class ValidationResult:
    is_valid: bool
    error_message: str | None
```

### 新規: `src/domain/calibration.py`

```
CalibrationActuatorProtocol  ← Protocol (HW抽象)
CalibrationConfig            ← dataclass (設定値)
CalibrationDetectionError    ← Exception (検出失敗)
CalibrationManager           ← ドメインロジック
```

## 電流スパイク検出アルゴリズム

機能設計書の定義に従う:
```
moving_avg = 移動平均（ウィンドウ幅: 5サンプル）
threshold = baseline_current * 1.5
if current > moving_avg + threshold:
    → 接触点またはフル位置に到達
```

- `baseline_current`: ウィンドウが満杯になった最初の移動平均（自由移動中）
- `moving_avg`: 直近5サンプルの移動平均（ウィンドウ満杯後は毎回更新）
- スパイク判定: `current > moving_avg + baseline * 1.5`

## キャリブレーション手順

```
_detect_zero(driver):
  1. home_return()
  2. _probe_contact(driver, start_pos=0)

_detect_full(driver, zero_pos):
  1. move_to_position(zero_pos)
  2. _probe_contact(driver, start_pos=zero_pos)

run_calibration(profile_id):
  1. _detect_zero(accel) → accel_zero
  2. _detect_full(accel, accel_zero) → accel_full
  3. home_return() (accel)
  4. _detect_zero(brake) → brake_zero
  5. _detect_full(brake, brake_zero) → brake_full
  6. home_return() (brake)
  7. _validate() → ValidationResult
  8. CalibrationResult を返す
```

## バリデーション項目
1. accel_full_pos > accel_zero_pos
2. brake_full_pos > brake_zero_pos
3. accel_stroke が [min_stroke, max_stroke] 範囲内
4. brake_stroke が [min_stroke, max_stroke] 範囲内

## テスト方針
- モックドライバで電流パターンを再現（スパイクあり/なし）
- バリデーションは独立したユニットテスト
- `CalibrationDetectionError` の送出確認
