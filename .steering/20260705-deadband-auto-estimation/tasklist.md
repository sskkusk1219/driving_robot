# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

---

## フェーズ1: パターンモデル

- [x] `src/models/learning_drive.py`: `PatternKind.ACCEL_DEADBAND_PROBE = "accel_deadband_probe"` 追加
- [x] 単体テスト: 既存`test_learning_drive.py`側で新kindの生成を網羅的にテスト(フェーズ2で実施)

## フェーズ2: 学習運転パターン生成

- [x] `src/domain/learning_drive.py`
  - [x] `ACCEL_DEADBAND_PROBE_PCTS`, `ACCEL_DEADBAND_PROBE_HOLD_S` 定数追加
  - [x] `BRAKE_HOLD_OPENINGS_PCT` に低開度4段(1.0, 2.0, 3.0, 5.0)を追加
  - [x] `LearningDriveConfig` に `accel_deadband_probe_pcts`/`accel_deadband_probe_hold_s` フィールド追加
  - [x] `generate_patterns`: CREEP_SETTLE直後・ACCEL_SWEEP開始前にACCEL_DEADBAND_PROBEパターン列を挿入
- [x] 単体テスト `tests/unit/test_learning_drive.py`
  - [x] ACCEL_DEADBAND_PROBEパターンがCREEP_SETTLE直後・ACCEL_SWEEP前に、指定開度昇順・hold_duration_sで生成されること
  - [x] max_accel_openingが低い場合に開度がクランプされること、0以下はスキップされること
  - [x] BRAKE_HOLDパターンに低開度4段が追加されていること(既存高開度4段も維持)
  - [x] 既存テストの回帰確認(29件通過。`test_contains_expected_kinds`/`test_hold_duration_uses_config`/`TestOrdering`を新パターン込みに更新)
  - [x] `learning_loop.py`側の回帰確認(状態機械は無変更、59件通過)

## フェーズ3: 不感帯自動推定ロジック

- [x] `src/domain/model_training.py`
  - [x] `DEADBAND_BIN_WIDTH_PCT`, `DEADBAND_SCAN_MAX_PCT`, `DEADBAND_ONSET_MARGIN_KMHS`, `DEADBAND_MIN_BIN_SAMPLES` 定数追加
  - [x] `_estimate_onset_deadband_pct(openings, response) -> float | None` 実装
  - [x] `estimate_dynamics_params`: accel/brake各スキャン用サンプル収集(セッション横断)、推定・上書きロジック追加
  - [x] docstring更新(「不感帯は推定対象外」の記述を削除し新ロジックを説明)
- [x] 単体テスト `tests/unit/test_model_training.py`
  - [x] `_estimate_onset_deadband_pct`: 明確なオンセットケース、オンセットなしケース、サンプル不足でNoneになるケース、ビン単位のサンプル不足スキップ
  - [x] `estimate_dynamics_params`: 十分なプローブ風合成データ(accel/brake双方)で不感帯が上書きされること
  - [x] `estimate_dynamics_params`: プローブデータが無い場合に既存値を保持すること(後方互換。既存テストを`test_deadbands_keep_current_when_no_probe_data`に改名・docstring更新)
  - [x] クランプ範囲([0, DEADBAND_SCAN_MAX_PCT])の検証(`test_result_never_exceeds_scan_max`)

## フェーズ4: 品質チェックと最終確認

- [x] `.venv/bin/python -m pytest tests/unit tests/integration` 全通過確認(916件通過)
- [x] `ruff check src/ tests/` 通過確認
- [x] `mypy`(変更3ファイル)通過確認
- [x] 実装後の振り返り(このファイルの下部に記録)

---

## 実装後の振り返り

### 実装完了日
2026-07-05

### 計画と実績の差分

**計画と異なった点**:
- design.mdの想定通り、`learning_loop.py`（状態機械）は一切変更せずに実装できた。`_Phase.MEASURE`の既存汎用ロジック（CREEPパターンと同型）と`BRAKE_HOLD`の既存スイープ機構をそのまま再利用する設計判断が奏功した。
- 既存テスト`test_deadbands_are_never_auto_estimated`は、そのままでは（テストデータが探索上限を超えるブレーキ開度=25%を使っていたため）新ロジックでも偶然パスしたが、テスト名・docstringが新しい仕様と矛盾するため`test_deadbands_keep_current_when_no_probe_data`に改名し、意図を「プローブデータが無ければ既存値保持」に明確化した。

**新たに必要になったタスク**:
- `_estimate_onset_deadband_pct`単体のテスト(TestEstimateOnsetDeadbandPctクラス)は、design.md執筆時には`estimate_dynamics_params`経由の統合的なテストのみを想定していたが、ビン単位のサンプル不足スキップやスキャン上限のクランプなど、関数内部のエッジケースを直接検証する方が信頼性が高いと判断し追加した。

### 学んだこと

**技術的な学び**:
- 学習運転の状態機械（`_Phase.MEASURE`）が既に「パターンの開度を無ランプで一定時間保持し、経過で次へ進む」という汎用的な形になっていたため、新しい種類のパターン(ACCEL_DEADBAND_PROBE)は状態機械のコード変更なしに追加できた。既存コードの抽象化が適切だと、新機能が「データ(パターン列)の追加」だけで実現できる好例。
- 不感帯推定のビン+オンセット検出方式は、既存のFOPDT同定(`pid_tuning.py`の`_segment_fopdt`)のオンセット検出パターン（車速がしきい値を超えるまでの時間で判定）と同じ設計思想（ベースラインとの差分をマージンで判定し、統計的信頼性を最小サンプル数で担保）を転用でき、コードベース全体で一貫した推定ロジックのスタイルになった。
- ブレーキ不感帯プローブは新しい状態機械ロジックを一切追加せず、既存の`BRAKE_HOLD_OPENINGS_PCT`という「ただの設定値リスト」に低い値を足すだけで実現できた。これは既存コードが「開度のリストをスイープする」という設計になっていたおかげで、機能追加のコストが極めて低くなった典型例。

### 次回への改善提案
- 実車での検証(学習サイクルを1回実行し、不感帯推定値が現実的な値になっているか確認)はまだ行っていない。特に「ACCEL_DEADBAND_PROBEの5段階(0.5/1.0/2.0/3.0/5.0%)が実際の車両の不感帯を挟み込めているか」は実測してみないと分からないため、初回の実機学習サイクル後に推定値を確認し、必要であれば`ACCEL_DEADBAND_PROBE_PCTS`の段階を調整すること。
- 低ブレーキ開度のBRAKE_HOLDは減速が緩く`brake_hold_timeout_s`(20秒)で打ち切られる可能性が高い。実機で学習運転の総時間が許容範囲か確認し、長すぎる場合はタイムアウトを短縮するか、低開度専用の短いタイムアウトを検討する。
