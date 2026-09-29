# 実行環境の問題— 2026-09-24

  `tests/`環境は`tests/`環境のみで完結しなければなりません。つまり、`src/`フォルダをまるごと削除しても`tests/`環境は問題なく動作しなければならない\
  現在の実行プロセスが`src/`を経由するならそれは、間違っている\
  確認したところ、tests/research の約30ファイルが src を import している。\
  `tests/`に必要なコードを`src/`から移植して、`tests/`環境のみでエラーなく動作すること

## 対応結果（2026-09-25 実施）

**ステータス: クローズ（2026-09-25 ユーザーが実機確認OK）**

`tests/research` の全 `.py`（本体＋テスト約60ファイル）から `src` の import をなくした。`src/` は編集していない。
方針は、使う部分だけを `tests/research` の平置きファイルへ**本番と同じロジックのまま**写す（各ファイル先頭に移植元を明記）。

| 持ち込んだもの | 置き場所（移植元） |
|---|---|
| 型・定数・小さな純関数（`FeedforwardParams`・`VehicleProfile`・`DriveLog`・`DrivingMode`・`LearningPattern`・`PreCheckResult`・`VEHICLE_STOP_SPEED_KMH`・`enforce_pedal_exclusion`・`to_jst_naive`・`ACTUATOR_PULSE_MAX` ほか） | `research_types.py`（`src/models/*`・`conversions`・`pedal_safety`・`utils/time`） |
| 学習パターン生成・`LearningDataError` | `learning_patterns.py`（`src/domain/learning_drive.py`） |
| 物理定数推定（`estimate_dynamics_params` と下請け・`COAST_CURVE_*`・`PEDAL_GAIN_MIN_OPENING_PCT`） | `dynamics_estimation.py`（`src/domain/model_training.py`） |
| FF の `predict_effort`（本番の忠実な写し） | `ff_model.py` の `FeedforwardModel`（`src/domain/control/feedforward.py`）。`ff_explain` の照合元 |
| `analytic_efforts`・`coast_accel` | `kaizen.py`（`src/domain/control/pedal_plan.py`。利用者は kaizen のみ） |
| 走行前チェック | `pre_check.py`（`src/domain/pre_check.py`） |
| アクチュエータ・CAN・UPS・設定読込み | `actuator_driver.py`・`can_reader.py`・`ups_monitor.py`・`app_settings.py`（`src/infra/*` をそのままコピー） |

**本番照合テストの扱い**（ユーザー決定: 固定値で埋める）
- `test_kpi_matches_production_monitor`: 本番 `KPIMonitor` に同じ系列を通した値（最大逸脱 3.1069116493567535・反転 9・p95 1.61）を固定。
- 新規 `test_predict_effort_matches_production_values`: 偽の線形回帰器で本番 `FeedforwardController` に通した 42 点の値を固定（停車保持・クリープ・惰行テーパ・制動・学習域クリップの全分岐）。
- `model_analysis` の「本番 `_metrics` との一致」は照合先がもともと `ff_model.metrics`（tests 側）だったので変更なし。

**副次的に直したもの**: `hardware.py` の実機ドライバ用ログフィルタが `logging.getLogger("src.infra.actuator_driver")` を指していた。移植先のロガー名（`tests.research.actuator_driver`）へ更新（そのままだと再送・クランプの正常ログが黙らなくなる）。

**再発防止**: `test_research_no_src_import.py`（AST で `src` の import を検出。関数内の遅延 import・`importlib.import_module("src…")` も対象）。

### 確認したこと（Claude 実行）

| 確認 | 結果 |
|---|---|
| `ruff check tests/research` | 合格 |
| `pytest tests/research`（通常） | 568 合格 / 2 失敗（下記） |
| **`src/` を除いたコピーで `pytest tests/research`** | **570 合格 / 2 失敗（同じ2件）** |
| `src/` 抜きコピーで手順1・手順3（スタブ、先頭20秒） | 完走（走行 CSV・グラフ・レポート出力まで） |
| `src/` 抜きコピーで `ff_explain --csv`（手順3 の CSV） | 完走。写した分岐と `predict_effort` の最大差 1.6e-14 % |

失敗2件は**今回の変更と無関係**（`src/` を消す前後で同じ）:
1. `test_stop_brake_floor_settings_are_validated`: YAML の `stop_brake_floor_offset_pct` が 0.5、テスト期待値が 8.0（5-1 で既知）。
2. `test_brake_ramp_starts_from_opening_at_phase_entry`（**新規に判明**）: `stop_brake_opening_pct` が 5-2 の再学習で 18.16→12.16 に変わり、テストの `assert _stop_return_brake_pct > 15.0` が成り立たなくなった。YAML が HEAD のままなら合格する。

**実機での動作確認**: 2026-09-25 ユーザーが実施しOK。

### ユーザーに実行してほしい確認

```bash
.venv/bin/ruff check tests/research
.venv/bin/python -m pytest tests/research -q                       # 失敗は上の2件だけのはず
.venv/bin/python -m pytest tests/research/test_research_no_src_import.py -q
# 実機（手順1＝初期化・走行前チェックまで。アクチュエータが動くので周囲に注意）
.venv/bin/python -m tests.research.main --only 1 --hw real
```
