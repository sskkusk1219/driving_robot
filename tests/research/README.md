# 研究開発用ハーネス（tests/research/）

`docs/Problem/ProblemReport_20260910.md` の手順 0〜10 を実行する。
本番の制御スタック（`src/`）は層が多く、どの層が KPI を壊しているか切り分けられないため、
ここでは **FF → PID → ペダル調停 → ペダル** だけの単純な系を組み、手順を 1 つずつ足しながら
プライマリー KPI（最大逸脱 ≤1.0 km/h・p95 ≤0.4 km/h・符号反転 ≤1回/5s）の達成度を測る。

## ファイル

| ファイル | 役割 |
|---|---|
| `main.py` | エントリポイント。引数でどこまで実行するかを決める |
| `config.py` | `config_testVehicle.yaml` の読み込み・検証・コメントを保った書き戻し |
| `config_testVehicle.yaml` | テスト用車両プロファイル。**ユーザーが直接編集してよい** |
| `hardware.py` | HW 層とスタブ（スタブ車両モデル含む）、初期化シーケンス（手順 1。本番から引用） |
| `pre_drive_check.py` | 走行前チェック（手順 1 と手順 2 の間。本番 `PreCheckRunner` ＋ブレーキを小刻みに踏んで停止確認） |
| `vehicle.py` | 開度の定義（原点 = 0%、9500 pulse = 100%）と YAML → 本番 `VehicleProfile` への変換 |
| `pedal_search.py` | ペダル探索（手順 2-0。不感帯と停車保持開度を車速応答で測る） |
| `pattern_drive.py` | パターン走行 → 2次多項式 FF モデル作成（手順 2。本番から引用） |
| `pedal_gain.py` | ペダルゲイン推定（手順 2-2。本番の計算で、使うサンプルのしきい値だけ「不感帯 + α%」に変える） |
| `coast_curve.py` | 惰行減速カーブの低速端の再同定（段2.5。低速だけ細ビンにする） |
| `creep_curve.py` | クリープ加速カーブの推定（段1） |
| `relearn.py` | 既存の走行 CSV からモデル・カーブだけを作り直すオフライン入口（実機不要。`--dry-run` で差分確認のみ） |
| `stop_decel.py` | 走行後の緩減速 → 停車保持（手順 2 のパターン走行の後。一方向に刻んで踏む） |
| `mode_drive.py` | モード走行（手順 3。走行モード管理の WLTP を FF だけで 50ms 周期で走る） |
| `mode_report.py` | モード走行のレポート（`reportYYYYMMDD_RunFF.md` ＋ 図）。CSV から作り直せる |
| `kpi.py` | プライマリー KPI の計算（最大逸脱・p95・符号反転。本番 `kpi_monitor.py` と同じ数え方） |
| `drive_log.py` | 走行ログ（走行前チェック〜停車保持を 1 本の CSV/PNG に記録） |
| `live_plot.py` | 走行グラフ（本番の自動走行画面と同じ 2 段レイアウト。別プロセスで描画） |
| `term.py` | 進捗の print（経過秒つき）、走行中の 1 行表示 |
| `results/` | CSV・図・レポートの出力先 |

ハードウェア設定（シリアルポート・CAN・GPIO）は本番と共通の `config/settings.toml` を読む。

## 実行

```bash
# 手順一覧
.venv/bin/python -m tests.research.main --list

# 手順 0（設定の作成・検証）
.venv/bin/python -m tests.research.main --upto 0

# 手順 1（初期化）をスタブで確認 / 手順 0→1 を通す
.venv/bin/python -m tests.research.main --only 1
.venv/bin/python -m tests.research.main --upto 1

# 手順 2（ペダル探索 → パターン走行 → FF モデル作成）をスタブで確認。HW を使うので手順 1 と一緒に
# （手順 1 と手順 2 の間に走行前チェックが自動で入る）
.venv/bin/python -m tests.research.main --steps 1,2

# 手順 3（FF のみで WLTP モード走行 → CSV/レポート）をスタブで確認。先頭 60s だけ走る
.venv/bin/python -m tests.research.main --steps 1,3 --limit-s 60

# 手順 0〜3 を通す / 手順 3 だけ / 手順 0 と 2
.venv/bin/python -m tests.research.main --upto 3
.venv/bin/python -m tests.research.main --only 3
.venv/bin/python -m tests.research.main --steps 0,2

# 実行対象の確認だけ（走らせない）
.venv/bin/python -m tests.research.main --upto 3 --dry-run
```

`--hw` の既定は `stub`（スタブ HW）。`--hw real` は**実アクチュエータが物理的に動く**ため、
ユーザーが自分で指定すること。

```bash
# 実機で初期化まで（周囲の安全を確認してから）
.venv/bin/python -m tests.research.main --upto 1 --hw real

# 実機で初期化 → 走行前チェック → パターン走行 → FF モデル作成（シャシダイナモ上で。0→約137 km/h まで加減速する）
.venv/bin/python -m tests.research.main --steps 1,2 --hw real

# 実機で初期化 → [準備] → 走行前チェック → FF のみで WLTP 1800s（シャシダイナモ上で。最高 131 km/h）
.venv/bin/python -m tests.research.main --steps 1,3 --hw real
```

終了コード: `0` 成功 / `2` 設定エラー / `3` 未実装の手順に到達 / `4` 初期化エラー /
`5` 走行エラー / `6` モデル作成エラー / `7` 走行前チェックエラー。

## 初期化シーケンス（手順 1）

本番 `src/app/robot_controller.py` の `start()` + `initialize()`、判定しきい値は
`src/domain/pre_check.py` から引用している。1 項目でも NG なら走行に進まない。

| # | 項目 | 内容 / 判定 | `checks.*` |
|---|---|---|---|
| 1 | 接続 | アクセル軸・ブレーキ軸・CAN を同時に開く | 対象外（常に実行） |
| 2 | Modbus 操作権 | PMSL コイルを **ブレーキ → アクセル** の順に有効化 | `init_servo_comm` |
| 3 | サーボ通信確認 | 両軸の現在位置を読み、往復通信を確認 | `init_servo_comm` |
| 4 | エラー消去 | 両軸アラームリセット → ALMC 再読で残留なしを確認 | `init_clear_errors` |
| 5 | サーボON | 両軸同時 | `init_servo_on` |
| 6 | CAN 通信チェック | 車速が読める（疎通のみ。停車判定はしない） | `init_can` |
| 7 | UPS 通信チェック | NUT から残量取得（>= 20%） | `init_ups` |
| 8 | アクチュエータ初期位置 | 両軸原点復帰 → 位置が ±10 pulse 以内 | `init_home_return` |

`config_testVehicle.yaml` の `checks:` セクションで項目ごとに `false` にすると、その項目は実行も
判定もせず結果表に `SKIP` と出る（例: UPS を使わない開発機では `init_ups: false`）。
サーボON・原点復帰など安全装置に関わる項目を外すと危険性が上がるため、YAML のコメントに注意書きがある。

終了時は成否にかかわらず **原点復帰 → サーボOFF → 切断** を必ず実行する
（ペダルを踏んだままプロセスを終わらせないため）。

## 走行前チェック（手順 1 と手順 2 の間）

実行順は **手順 1（初期化）→ 走行前チェック → 手順 2 以降（走行）**。走行する手順の最初の 1 つの前に
自動で 1 回だけ行う（`--dry-run` / `--list` にも表示される）。本番 `RobotController` の arm と同じ 3 段で、
判定は本番 `src/domain/pre_check.py` の `PreCheckRunner` をそのまま使う。
場所は`src/domain/control/conversions.py` 変数名`VEHICLE_STOP_SPEED_KMH: float = XXX`

| # | 段 | 内容 / 判定 | `checks.*` |
|---|---|---|---|
| 1 | 踏込前チェック | 通信確認・サーボ状態（アラーム）・プロファイル・UPS残量・アクチュエータ位置（原点 ±10 pulse） | `pre_communication` / `pre_servo_state` / `pre_profile` / `pre_ups` / `pre_actuator_position` |
| 2 | 停止確認 | ブレーキを `pedal_search.step_mm`（0.5mm）ずつ踏み、刻むたびに `dwell_s`（1s）の平均車速を読む。**`VEHICLE_STOP_SPEED_KMH`（0.02 km/h）未満で停止確認**してその位置で保持。車速が下がっている間は踏み増さない | `pre_brake_stop`（false ならブレーキを踏まず現在位置のまま次へ） |
| 3 | 踏込後チェック | 通信確認・サーボ状態・プロファイル・UPS残量・**車速確認（0.02 km/h 未満）** | `pre_communication` / `pre_servo_state` / `pre_profile` / `pre_ups` / `pre_vehicle_stopped` |

- `checks.pre_*` で `false` にした項目は判定せず `[SKIP]` と表示する（`pre_communication` 〜 `pre_ups`
  は踏込前・踏込後の両方に効く）。`pre_ups: true` は `init_ups: false` のときは設定できず、手順 0
  （設定検証）で NG になる。
- 本番との違い: 2 で `stop_brake_opening_pct` を一気に踏まない（クリープ中に一気に踏むのは危険なため）。
  キャリブレーション項目は除く（tests 環境では使わない）。
- 始めから車速が 0.02 km/h 未満なら、踏まずに停止確認とする（次の 2-0 ペダル探索でブレーキを離すため）。
- `pedal_search.brake_max_pct`（50%）まで踏んでも停車しない、または NG 項目があれば終了コード 7 で止まり、
  走行には進まない（終了処理で原点復帰）。
- スタブ HW はシャシダイナモ上でクリープ中（`feedforward.creep_speed_kmh`）から始まるので、この段を実際に通る。

## パターン走行 → FF モデル作成（手順 2）

最初にペダル探索（2-0）で不感帯と停車保持開度を測り、そのあと本番の学習運転パターンを走る。
パターン生成・モデル学習は本番の関数をそのまま呼ぶが、走行ループ自体は `pattern_loop.py` に
アルゴリズムを移植した自前実装（`PatternLoop`）で、本番 `LearningLoop` クラスは実行しない
（`/src` の本番コードを実行しないという遵守事項に対応）。

| 段 | 内容 | 引用元 |
|---|---|---|
| 2-0 探索 | クリープ中に 0.5mm ずつ踏み、アクセル/ブレーキの不感帯と停止確認開度を測る → 停止確認 +10% で停車保持 | 研究用（下記） |
| 2-1 走行 | パターン列（クリープ・不感帯プローブ・加速スイープ・定常ブレーキ・コーストダウン・高速巡航トリム）を 100ms 周期で実行。不感帯プローブ・定常ブレーキ・巡航トリムの開度は 2-0 の不感帯 + offset | `LearningDriveManager.generate_patterns` / `pattern_loop.PatternLoop`（`LearningLoop` のアルゴリズムを移植） |
| 走行後 | アクセル 0% → 不感帯の手前まで一発 → 0.5mm 刻みで 0.2G 付近の緩減速 → 停車保持 | 研究用（下記。本番 `_decelerate_to_stop` の置き換え） |
| 2-2 モデル | 2次多項式＋標準化＋Ridge の逆モデル（アクセル/ブレーキ）→ クリープ・惰行減速カーブ・ペダルゲインを推定（不感帯・停車保持は 2-0 の実測を採用。ペダルゲインは `pedal_gain.py` で不感帯 + 0.5% 以上のサンプルから推定し直す） | `train_inverse_model` / `estimate_dynamics_params` |

- 走行ログは走行前チェックから停車保持までを 1 本にした `results/drive_log_<stub|real>_<日時>.csv` と同名の `.png`
  （下記「走行ログ」）。モデル作成はそのうちパターン走行の行だけを使う。
- モデルは `results/models/<車両名>_<日時>.pkl`。**実機走行のときだけ** `feedforward.model_path` と物理定数を
  `config_testVehicle.yaml` へ書き戻す（スタブ走行の模擬値で実機の値を上書きしないため）。
- 開度の定義: **0% = 原点復帰位置（0 pulse）、100% = ストローク限界 9500 pulse**（本番 `_ACTUATOR_PULSE_MAX`）。
  本番のキャリブレーションは使わない。原点からペダルに触れるまでの隙間とペダルの遊びは不感帯に含まれる。
- ペダルの固定開度は「2-0 で測った不感帯 + `learning` の offset」。本番の開度はキャリブレーション前提の絶対値で、
  原点から測ると遊びの中に入る（実機でブレーキ 1〜10%・巡航トリム 1.5/3.0% は惰行と同じだった）。

  | パターン | 開度 | YAML キー（既定） | 本番 |
  |---|---|---|---|
  | アクセル不感帯プローブ | アクセル不感帯 + offset | `accel_deadband_probe_offsets_pct`（0.5/1/2/3/5） | 0.5〜5% |
  | 高速巡航トリム | アクセル不感帯 + offset | `cruise_trim_offsets_pct`（1/2/3） | 1.5/3.0% |
  | 定常ブレーキ（BRAKE_HOLD） | ブレーキ不感帯 + offset | `brake_hold_offsets_pct`（0.5〜4 を 0.5 刻み） | 1〜40% |

- ペダルゲインの推定に使うのは、開度 ≥ 不感帯 + `accel_gain_min_offset_pct` / `brake_gain_min_offset_pct`（既定 0.5%）の
  サンプル（本番は +5%）。この車両のブレーキは 16% ≈ 0.2G・20% ≈ 0.41G と効きが急で、+5% ≈ 0.38G は減速G ガバナーの
  上限ぎりぎりのため定常サンプルが採れず、ブレーキゲインが未同定だった。モデル作成時に本番条件の推定値も並べて表示する。
  隙間を大きく取った設置でも効き始めを跨ぐため。
- 本番との違い: DB/セッション/WebSocket なし。FOPDT 同定・SIMC の PID 初期ゲインは計算しない（手順 4/6/8 で適合）。
  走行後の緩減速は本番の「0.1s ごとに ±1%」ではなく `stop_decel.py` で行う（下記）。
  ペダルの固定開度とペダルゲイン推定のしきい値は不感帯基準（上記）。
  走行前の「停車保持ブレーキを一気に踏む → 停車待ち」は、走行前チェックでブレーキを小刻みに踏む方式に置き換えた。

### 2-0 ペダル探索（`pedal_search.py`）

いきなり停車保持開度を踏まず、1 刻み（`pedal_search.step_mm` 0.5mm ≈ 0.53%）ずつ踏んで `dwell_s` 待ち、
その間の平均車速で判定する。

| # | 段 | 判定 |
|---|---|---|
| 1 | クリープ安定待ち | 走行前チェックのブレーキを離し、両ペダル原点で、`creep_window_s` 平均の傾き < `creep_settle_kmhs` かつ `creep_min_speed_kmh` 以上・`creep_settle_min_s` 経過後 → 基準車速 |
| 2 | アクセル探索 | 基準 + `onset_margin_kmh` を `confirm_count` 刻み連続で超えた最初の位置 = アクセル不感帯 → 原点へ戻して 1 をやり直す |
| 3 | ブレーキ探索 | 基準 − margin を連続で割った最初の位置 = ブレーキ不感帯。効き始めた後は車速が下がっている間は踏み増さず、0.02 km/h 未満になった位置 = 停止確認開度 |
| 4 | 停車保持 | 停止確認開度 + `stop_hold_margin_pct`（10%）まで刻んで踏み、保持 = `stop_brake_opening_pct` |

- 実機のときだけ 3 値（`accel_deadband_pct` / `brake_deadband_pct` / `stop_brake_opening_pct`）を走行前に YAML へ保存する。
  2-2 の `estimate_dynamics_params` の推定値は表示のみで書き戻さない。
- 失敗（クリープしない・上限まで踏んでも反応しない・停車しない）は終了コード 5。
- 検出位置は車速の立ち上がり遅れのぶん真値より深く出る（スタブ・1mm 刻み・0.3s 待ちで +1.4% 程度）。
- 連続移動にしないのは、ブレーキの最低速度 10mm/s で踏み続けると、むだ時間 0.6〜0.9s と停車までの数秒の間に
  大きく踏み過ぎるため。

### 走行後の緩減速（`stop_decel.py`）

本番 `_decelerate_to_stop` は 0.1s ごとに「車速差 ÷ 0.1s」で減速度を出し、0.2G との大小でブレーキを ±1% 動かす。
実機では 0.1s 差分のノイズ（標準偏差 2〜3 km/h/s）とブレーキ→減速の遅れ（効き始め約 0.3s、ピーク 0.6〜1.0s）で
押し戻しを繰り返したため、一方向に刻んで踏み、行き過ぎそうなら待つ方式にした（設定は `decel_stop` セクション）。

| # | 段（CSV の phase） | 内容 |
|---|---|---|
| 1 | `APPROACH` | アクセルを原点へ。ブレーキを `brake_deadband_pct − approach_margin_pct`（13.68 − 1%）まで最高速度で動かす（遊びの中なので減速は出ない） |
| 2 | `STEP` | `step_mm`（0.5mm）踏む → `dwell_s`（1s）待つ → 直近 `slope_window_s`（1s）の車速の傾き（最小二乗）で減速度を出す。`target_decel_g − press_margin_g`（0.18G）未満なら 1 刻み踏み増し、`release_above_g`（0.3G）を超えたときだけ 1 刻み戻す。それ以外は保持 |
| 3 | `STOP_HOLD` | 車速が `VEHICLE_STOP_SPEED_KMH`（`src/domain/control/conversions.py`）未満で停止確認 → 停車保持開度まで刻んで踏む（通常は到達済み） |

- 踏み増しの上限は `feedforward.stop_brake_opening_pct`。届いても減速が足りなければ「上限で待機」して停車を待つ。
- 刻むたびに 車速 / 減速度 G / ブレーキ位置 / 判定（踏み増し・保持・戻し・上限で待機）を print し、
  最後に所要時間・最大減速度・踏み増し/戻しの回数を出す。
- `timeout_s`（60s）を超えても停車しない、または CAN 車速が読めなければ終了コード 5（終了処理で原点復帰）。
- 実機の効き（約 120 km/h）: 14% まで惰行と同じ、16% ≈ 0.2G、20% ≈ 0.41G。効き始め以降 1% ≈ 0.065G
  （0.5mm 刻み ≈ 0.035G）。最低速度の連続移動では遅れの間に 0.35〜0.6G まで行き過ぎる見込みのため刻みにしている。

## FF のみでモード走行（手順 3）

実行順は **手順 1（初期化）→ 準備 → 走行前チェック → 手順 3**。準備（走行モードを DB から、FF モデルを
`feedforward.model_path` から読む）は HW に触る前に行い、失敗したら終了コード 2 で止まる。

```
基準車速 → FF（手順 2 のモデル）→ effort → 調停（符号で振り分け）→ アクセル / ブレーキ → 車両 → 車速
```

| 項目 | 内容 | 引用元 |
|---|---|---|
| モード | `modes.wltp_mode_name`（`01_WLTP_Low,Mid,Hi,ExHi`、1800s・1Hz の点列）を線形補間 | 本番 `DriveLoop._ref_speed_at` |
| FF | 現在・先読み（0.5/1/2/3s）・過去（0.5/1s）の基準車速から effort を出す | 本番 `FeedforwardController.predict_effort`（関数として呼ぶ） |
| PID | なし（Kp=Ki=Kd=0。手順 5 以降で足す） | — |
| 調停 | effort > 0 → アクセル、< 0 → ブレーキ、開度上限でクランプ。不感帯補償・レートリミット・ヒステリシスなし（`arbiter.enable_*` が true なら開始しない） | 研究用（第1段階） |
| 周期 | 50ms（`control.loop_interval_ms`）。変化した軸だけ位置指令 → 電流を読む。1 周期以上遅れたら次から数え直し、回数をレポートに出す | 本番 `DriveLoop` の 1 サイクル |
| 安全停止 | CAN 読み取り失敗・軸通信失敗・過電流・最高速超え・逸脱（`vehicle.stop_deviation_*`）→ 両ペダルを離して中断（終了コード 5。途中までのレポートは作る） | 本番 `DriveLoop` |
| 減速G ガバナー | 0.4s の車速の傾きが `vehicle.max_decel_g` × 0.98 以上ならブレーキを頭打ち → 2%/周期ずつ下げる（`mode_drive` セクション）。**本番の自動走行には無い安全網**なので作動時間をレポートに出す | `pattern_loop.PatternLoop` と同じ規則 |
| 終わり | 停車していれば停車保持開度を保持。止まっていなければ（`--limit-s` で打ち切ったとき）緩減速で停車 | `stop_decel.py` |

- ターミナル: 1s ごとに `t 基準 実 偏差 Kp Ki Kd | アクセル ブレーキ | FF 区間 max|偏差| 進捗` を 1 行。区間が変わると見出し。
- グラフ: 走行中は本番と同じ 2 段（基準車速＋実車速 / アクセル＋ブレーキ）。
- CSV: 走行前チェックから停車保持までの `results/drive_log_<stub|real>_<日時>.csv`（0.1s 刻み）。
  モード走行の行は `section = MODE_DRIVE`、`mode_time_s` = モード経過秒、`pattern` = WLTP 区間名、
  `phase` = その周期のペダル（`ACCEL` / `BRAKE` / `BRAKE_GOV`（ガバナー作動）/ `COAST`）。
- レポート: `results/reportYYYYMMDD_RunFF.md` と図のフォルダ `results/reportYYYYMMDD_RunFF/`
  （同じ日の 2 本目以降は `_2`, `_3`…）。内容は KPI 判定表・実施条件・全体図・偏差図・
  WLTP 区間別 / 走行状態別（加速・定速・減速・停車）/ 基準車速の帯別の表・1.0 km/h 超えの区間・
  最大逸脱付近の拡大図・ペダル指令の特徴（不感帯より浅い指令の時間など）・所見。考察は走行後に追記する。
- KPI は MODE_DRIVE の 0.1s 刻みの行（CSV と同じ行）で計算する（`kpi.py`）。本番 `KPIMonitor` と
  最大逸脱・符号反転は一致、p95 は本番がビン上端のため最大 +0.01 km/h 違う（テストで確認）。
- レポートの作り直し: `.venv/bin/python -m tests.research.mode_report <CSV> --label FF`
- `--limit-s S` はモードの先頭 S 秒だけ走る動作確認用（KPI は参考値。レポートにも書く）。

## 走行ログ（`drive_log.py`）

走行前チェックから手順 2 の停車保持までを **1 本の CSV/PNG** に残す。

- ファイル: `results/drive_log_<stub|real>_<日時>.csv` と同名の `.png`。走行中のグラフは DISPLAY があればウィンドウ、
  無ければ `results/live_drive.png` を 2s ごとに上書き。
- 列（2026-09-13 A7 から。**すべての区間で同じ列・同じ意味**）:

| 分類 | 列 | 中身 |
|---|---|---|
| 時間 | `timestamp` / `elapsed_s`（走行前チェック開始 = 0）/ `mode_time_s`（モード走行だけ）/ `cycle_ms` | `cycle_ms` はその行の読み取り（パターン・モード走行は 1 周期の処理）にかかった実時間 |
| 車速 | `ref_speed_kmh` / `actual_speed_kmh` / `deviation_kmh` | 基準の無い区間は基準・偏差が空欄 |
| 指示開度 FF | `accel_ff_pct` / `brake_ff_pct` | FF の出力をペダル別に分けた値（モード走行だけ） |
| 指示開度 FF・PID | `accel_cmd_pct` / `brake_cmd_pct` / `accel_cmd_mm` / `brake_cmd_mm` | アクチュエータへ送った最終指令（ガバナー・待機位置を含む）。サンプラーの区間は各軸へ最後に送った目標位置 |
| 実開度 | `accel_actual_pct` / `brake_actual_pct` / `accel_actual_mm` / `brake_actual_mm` | 0x9000 から 14 レジスタのまとめ読みの PNOW（読めなかった行は空欄） |
| 電流 | `accel_current` / `brake_current` | CNOW [mA]（同じまとめ読み） |
| ステータス | `section` / `pattern` / `phase` / `governor_active` / `alarm_accel` / `alarm_brake` / `{accel,brake}_servo_on` / `_moving` / `_pos_done` / `_alarm_code` | サーボON = DSS1 SV、移動中 = DSSE MOVE、位置決め完了 = DSS1 PEND、アラームコード = ALMC |

- 旧形式（〜A6: `accel_opening` / `accel_pos` / `ff_effort_pct` など）の CSV も、解析スクリプトは
  `drive_log.cmd_opening` / `ff_effort` などを通して読める。A7 の走行の確かめは `debug_a7`。

| section | 区間 | phase の例 | 記録の仕方 |
|---|---|---|---|
| `PRE_DRIVE_CHECK` | 走行前チェック | `PRE_CHECK` / `BRAKE_STEP` / `POST_CHECK` | サンプラー |
| `PEDAL_SEARCH` | 2-0 ペダル探索 | `CREEP_WAIT` / `ACCEL_SEARCH` / `BRAKE_SEARCH` / `STOP_HOLD` | サンプラー |
| `PATTERN_DRIVE` | 2-1 パターン走行 | PatternLoop の phase（`pattern` 列に「番号:種類」） | PatternLoop の on_sample コールバック |
| `MODE_DRIVE` | 手順 3 のモード走行 | `ACCEL` / `BRAKE` / `BRAKE_GOV` / `COAST` | 50ms 制御ループが 2 周期ごと |
| `DECEL_TO_STOP` | 走行後の緩減速〜停車保持 | `APPROACH` / `STEP` / `STOP_HOLD` | サンプラー |

- サンプラーは `output.csv_interval_s`（0.1s）ごとに CAN 車速と両軸のまとめ読みを記録する。
  パターン走行中は 100ms の制御ループと Modbus を取り合わないよう止める。
- 保存は停車保持の後（モデル作成の前）。異常終了でも、原点復帰の後にそこまでのログを保存する。
- 2-2 のモデル作成は `section = PATTERN_DRIVE` の行だけを読む（`section` 列の無い旧 `pattern_drive_*.csv` は全行）。

## 既存ログからの再学習（`relearn.py`）

実機を走らせ直さずに、既存の走行 CSV（手順2 の `results/drive_log_real_*.csv`）からモデル・
物理定数（惰行カーブ等）だけを作り直せる（段2.5。実機を走り直すと不感帯・クリープ速度も
同時に変わってしまい 1 変数比較にならないため）。

```bash
.venv/bin/python -m tests.research.relearn tests/research/results/drive_log_real_XXXXXXXX.csv --dry-run  # 差分確認のみ
.venv/bin/python -m tests.research.relearn tests/research/results/drive_log_real_XXXXXXXX.csv            # config_testVehicle.yaml へ保存
```

## 設定の書き戻しについて

手順 2/4/6/8 は同定・適合したパラメータを `config_testVehicle.yaml` へ書き戻すが、
**値の行だけ**を差し替えるためコメント・行順・インデントは保たれる。
リストは 1 行の flow 形式（`[a, b, c]`）で書くこと（複数行の `- a` 形式は書き戻しが拒否する）。

## テスト

```bash
.venv/bin/python -m pytest tests/research -q
```

`config_testVehicle.yaml` はユーザーが実機に合わせて書き換えるファイルなので、テストはその値を
決め打ちしない。スタブ車両の物理に依存するテストは `dataclasses.replace` で固定値を明示する
（`test_research_pedal_gain.py` の `_params()` 参照）。
