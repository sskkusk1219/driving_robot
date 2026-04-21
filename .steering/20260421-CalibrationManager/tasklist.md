# CalibrationManager タスクリスト

## タスク一覧

- [x] T1: `src/models/calibration.py` に `ValidationResult` dataclass を追加
- [x] T2: `src/domain/calibration.py` を新規作成（CalibrationManager 本体）
- [x] T3: `tests/unit/test_calibration.py` を新規作成（ユニットテスト）
- [x] T4: テスト・lint・型チェックを実行して全パスを確認

## 申し送り事項

- **実装完了日**: 2026-04-21
- **テスト**: 23件全パス (pytest), ruff lint クリーン, mypy strict クリーン
- **計画との差分**: なし。設計通りに実装完了
- **注意点**:
  - `MockActuatorDriver._make_iter` は `@staticmethod` の async generator。`AsyncIterator[float]` の型注釈に `# type: ignore[override]` が必要
  - `asyncio.sleep(0.0)` でテスト高速化（`step_interval_s=0.0` を CalibrationConfig に設定可能）
  - `_probe_contact` の baseline は「ウィンドウ満杯になった最初の移動平均」。baseline 確定後の次サンプルからスパイク判定開始
- **次回への改善提案**:
  - `RobotController.run_calibration()` の stub を `CalibrationManager` の実際の呼び出しに差し替えること
  - 実機チューニングで `CALIB_MIN_STROKE_PULSE` / `CALIB_MAX_STROKE_PULSE` の値を調整すること

