# タスクリスト: MVP機能動作確認

## タスク

- [x] T1: stubs.py の _StubActuator に enable_modbus_control を追加（mypy修正）
- [x] T2: src/infra/gpio_monitor.py:6 の行長エラーを修正（ruff E501）
- [x] T3: tests/hardware/test_emergency_stop.py:39 の行長エラーを修正（ruff E501）
- [x] T4: tests/hardware/test_emergency_stop_home_return.py:29 の行長エラーを修正（ruff E501）
- [x] T5: 全ユニットテスト・lint・型チェックが通ることを最終確認

---

## 申し送り事項

**実装完了日**: 2026-05-12

### 計画と実績の差分
- 計画通り T1〜T5 を全て完了
- 実装バリデーション中に `functional-design.md` の記述漏れ（`enable_modbus_control`）を追加で修正した

### 発見した知見
1. `_StubActuator` は `ActuatorDriverProtocol` を完全実装する必要がある。プロトコルにメソッドを追加した際はスタブも必ず同期させること
2. 日本語文字列を含む行は ruff E501 が文字幅として CJK を2カウントする場合があり、100文字制限に引っかかりやすい
3. `functional-design.md` の `ActuatorDriver` に `move_to_position` の周期が「10ms」と誤記されていた（50ms が正）。今回の修正で合わせて修正済み

### 本番環境での動作確認について
- ユニットテスト (392件) / lint / mypy はすべてパス ✅
- ハードウェアテストは実機接続が必要（`tests/hardware/` 以下のスクリプトを手動実行）
  - 非常停止スイッチ確認: `tests/hardware/test_emergency_stop.py`
  - 非常停止→原点復帰 E2E: `tests/hardware/test_emergency_stop_home_return.py`
  - 過電流→原点復帰 E2E: `tests/hardware/test_overcurrent_home_return.py`
  - キャリブレーション（手動ジョグ）: `tests/hardware/test_calibration.py`
  - 実行方法: `.venv/bin/python tests/hardware/[スクリプト名]`
