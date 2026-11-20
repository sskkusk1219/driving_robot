# src/robot — ロボット制御（tests/research の移植）とランナー

`src/robot/core/` は tests/research の**写し**（`scripts/port_research.py` が作る。手で直さない。一覧は
`core/PORTED.md`）。それ以外の `src/robot/` は src だけのコードで、core を直さずに使う・差し替える。

| ファイル | 内容 |
|---|---|
| `config_store.py` | 共通 yaml ＋ 車両 yaml の合成・保存の写し（段1）。`save_vehicle_values` は車両キーの直接書き換え |
| `run.py` | ランナー（`python -m src.robot.run`）。core の `run_steps` を呼ぶ |
| `events.py` | イベント（JSON 1 行）の形式と出し口 |
| `streaming_log.py` | `StreamingSessionLog`（core の `SessionLog` の派生。sample/csv イベント） |
| `patches.py` | core の挙動を差し替える所（**差し替え点はここだけ**） |
| `emergency_stop.py` | 走行するジョブの非常停止の流れ（減速して停車 → N → スタート 10 秒 → 原点復帰。差し替え 10） |
| `interrupt.py` | SIGINT / SIGTERM の扱い（終了処理を必ず走らせる） |
| `interactive.py` | 対話ジョブ（手動運転・min 測定） |
| `schedule_spec.py` | タイムスケジュール再生の純粋な部分（データ・補間・頭打ち（同時踏みは許す）・ボタン名の検査。core を import しない） |
| `plan_job.py` | 計画だけを作るジョブ（`--plan-only`）と、作った計画を手順 24 へ渡す部分（`--plan-in`。段4） |
| `schedule_job.py` | タイムスケジュールの開ループ再生（段6。core の `Step` として `run_steps` に載せる） |
| `serve.py` | 常駐ランナー（`--serve`。初期化したらサーボ ON・接続のまま、標準入力のコマンドでジョブを走らせる。下の「常駐ランナー」） |

## ランナーの使い方

```
.venv/bin/python -m src.robot.run --vehicle test_vehicle --steps 1,24 --hw stub --limit-s 60
.venv/bin/python -m src.robot.run --vehicle test_vehicle --steps 1,24 --mode 09_US06
.venv/bin/python -m src.robot.run --vehicle test_vehicle --interactive          # 手動運転
```

| 引数 | 内容 |
|---|---|
| `--vehicle 車ID` | `vehicles/<車ID>/vehicle.yaml`（必須） |
| `--steps 24,25` | 手順の並び（手順 1 を含まなければ、ランナーが `quiet_connect.py` で接続・サーボ ON だけ裏で行う。初期化の表・init イベントなし）。解釈は core の `resolve_steps` と同じ。動かせるのは 1・2・12〜14・20〜25（**手順 3 は動かさない**。src のモード走行は常に手順 24） |
| `--serve` | 常駐ランナー（下の「常駐ランナー」。`--steps`・`--interactive`・`--schedule*`・`--plan-only` と排他。`--mode`・`--limit-s`・`--plan-in`・`--no-pre-check` はコマンドで指定する） |
| `--interactive` | 手順 1（初期化）だけ行い、標準入力の JSON 1 行コマンドを受け付ける（`--steps` とは排他） |
| `--hw stub｜real` | 既定 stub。real は実アクチュエータが動く |
| `--mode ID｜名前` | 手順 24 で走る走行モード（DB の `driving_modes` の ID か名前。合成モード名も可）。学習の手順（2・12〜14）と同じ実行では指定できない |
| `--limit-s S` | モード走行を先頭 S 秒で打ち切る（動作確認用） |
| `--schedule ID｜名前` | タイムスケジュール（`time_schedules`。環境変数 `DATABASE_URL` があればその DB）を開ループで再生する（`--steps`・`--interactive` と排他。`--mode`・`--limit-s` は使えない） |
| `--schedule-file JSON` | 同じ。スケジュールを JSON（`schedule_spec.schedule_to_dict` の形）で渡す。Web 側はこちらを使う（検査済みの内容をそのまま走らせ、DB なしの環境でも動かすため） |
| `--plan-only --mode 名 --plan-out PATH` | **ハードに触れず**、モードの計画（`ModeDriveSetup`。作成に約 30 秒）だけを作って pickle で書く（`{version, vehicle, mode, created, setup}`。一時ファイル→置き換え）。`--steps` などとは同時に使えない。イベント `state phase=planning` → `planned`。終了コード 0 / 2（設定・モード）/ 5（予期しない）/ 130（SIGINT。書きかけは消す）。Web の「開始」ポップアップの間に使う |
| `--plan-in PATH` | 手順 24 の準備（計画の作成）を、ファイルから読む（0 秒）に差し替える。`--mode` と手順 24 が要る。車ID・モード名が違えば終了コード 2 |
| `--no-pre-check` | 走行前チェックを飛ばす（シーケンスの 2 段目以降。「走行前チェック: シーケンスの 2 段目以降のため行いません」と言う） |
| `--vehicles-dir` / `--common` | 車両フォルダの親（既定 `vehicles/`）／共通 yaml（既定 `config/robot_common.yaml`） |
| `--events-fd N` | イベントを出す fd。無ければ標準出力に `@@EV ` 前置きで出す |

実行前に止める誤り（終了コード 2・HW に触らない）: 動かせない手順・`--limit-s` が正でない・
手順 3 など動かせない手順・`--mode` の使い方の誤り・手順 23 と 24 の同時指定。
ボタン操作は常に有効（`buttons.enabled` は廃止。旧 yaml に残っていても無視する）。

### 終了コード

core の `run_steps` と同じ。ランナーが足すのは 130・143 だけ。

| コード | 意味 |
|---:|---|
| 0 | 成功 |
| 2 | 設定エラー |
| 3 | 未実装の手順に到達 |
| 4 | 初期化エラー |
| 5 | 走行エラー |
| 6 | モデル作成エラー |
| 7 | 走行前チェックエラー |
| 8 | 手順 2 の学習結果のチェックが NG（yaml は書き換えていない） |
| 130 / 143 | SIGINT / SIGTERM で中断（終了処理は済んでいる） |

### CSV の置き場所

`output.results_dir` = `vehicles/<車ID>/data/`。学習データ（手順 2・12・13・14 が後で読み直す材料。
手順 13・14 は pkl の meta `sources` の CSV をここから探す）はそのまま残す。走行ログ（モード走行の CSV）は
Web 側が DB へ入れた後に消す（段3）。ランナーは `csv` イベントで「パスと種類」を知らせるだけ。
合成 yaml は `vehicles/<車ID>/.work/merged.yaml`。走行時の設定のスナップショット（CSV と同名の `.yaml`）は
core が CSV の隣に書く（Web 側が CSV と一緒に消す）。

## イベント

標準出力の行頭が `@@EV ` の行（`--events-fd` なら fd の各行）が 1 イベント。JSON オブジェクトで、
共通の項目は `type`（種類）と `ts`（壁時計の epoch 秒）。それ以外の行は core の `say()` の行
（`[  12.3s] …`）でそのまま「現在ステータス」に出せる。形式の読み書きは `events.py`
（`decode_event_line`）。

| type | 項目 | いつ |
|---|---|---|
| `state` | `phase`（下の表）・`step`・`title`・`ok`・`error`・`interrupted`・`exit_code` | 状態が変わるたび |
| `init_step` | `name`・`status`（`ok`/`ng`/`skip`）・`detail` | 初期化の 1 項目ごと（接続の後〜原点復帰） |
| `sample` | `t_s`（経過秒）・`mode_time_s`・`ref_kmh`（基準車速。無い走行は null）・`speed_kmh`・`accel_cmd_pct`・`brake_cmd_pct`（指令開度）・`accel_actual_pct`・`brake_actual_pct`（実開度。読めなければ null）・`section`・`pattern`（モードの区間名。WLTP は Low/Mid/High/ExHi）・`phase` | 走行ログを 1 行記録するたび。**5 Hz に間引く** |
| `csv` | `path`（絶対パス）・`kind`（`learning`/`drive`）・`rows` | CSV を保存した直後 |
| `result` | `exit_code`・`meaning`・`error`（最後に手順を止めた例外の文。手順 2 の学習結果チェック NG ならその理由。なければ null）・`elapsed_s` | 最後に 1 回（必ず出る。`state done` の直後） |
| `button` | `button`（`start`/`P`/`N`/`D`）・`phase`（`start`/`end`）・`seconds`（押す秒数） | ボタンを押し始めたとき（`start`）と、`seconds` 後（`end`）。ボタンなしでも出る（段11。画面のペダルの図に矩形で出す。JobManager が最新 sample の時刻と対にして持つ） |
| `ack` | `id`・`cmd`・`ok`・`message`（`--serve` ではジョブのコマンドにも `job_id`） | 対話ジョブのコマンドの結果（`--serve`: すべてのコマンドの受理・拒否） |
| `watch` | `level`（`warn`/`error`）・`detail` | `--serve` の待機中の見張り（下の「常駐ランナー」）。`error` はブレーキを停車保持の開度で保ち、サーボ ON のまま閉じて終了する |
| `status` | `id`・`accel_pct`・`brake_pct`・`accel_pulse`・`brake_pulse`・`speed_kmh`・`min_*_pct`・`max_*_pct` | 対話ジョブ（status コマンド・動かした後・開始時） |

`state` の `phase`:

| phase | 意味 |
|---|---|
| `init` | 手順 1（初期化中）の開始 |
| `prepare` | 走行の手順の準備（走行モード・FF モデルの読み込み） |
| `pre_check` | 走行前チェック |
| `step` | 手順 `step` の開始 |
| `step_end` | 手順の終わり（`ok`。失敗なら `error`・中断なら `interrupted`） |
| `ready` / `d` / `n` | 学習の手順（2・12〜14）のボタン操作。Ready（スタート短押し）・D は最初の学習の手順の前、N は最後の後（D-14b） |
| `shutdown` | 終了処理（原点復帰・サーボOFF・切断） |
| `done` | 完了（`exit_code`。この後に `result`） |
| `interactive` | 対話ジョブがコマンド待ちになった |
| `idle` | `--serve`: 次のコマンドを待つ（`needs_init`。`job_id` なし） |
| `job_end` | `--serve`: ジョブの終わり（`mode`=`home`/`keep`。サーボ ON・接続のまま） |

`csv` の `kind`: モード走行の行（`section=MODE_DRIVE`）を含めば `drive`（Web が DB へ入れて消す）、
それ以外（ペダル探索・G ランプ・FEL・走行前チェックだけ）は `learning`（残す）。

`init_step` は core の `InitReport.add / skip` を包んで出す（say の行もそのまま出る）。

## 常駐ランナー（`--serve`、2026-11-20）

ジョブごとにランナー（別プロセス）を起動すると、core の `run_steps` が終わりに必ず shutdown（原点復帰 → サーボ OFF → 切断）するため、
シーケンスの段の間に接続し直し（約 5 s）が入る。そこで**初期化を 1 回行ったら動かし続け**、Web サーバー（JobManager）が標準入力の
JSON 1 行コマンドでジョブを送る。本体は `serve.py`。

```
.venv/bin/python -m src.robot.run --vehicle test_vehicle --serve --hw stub
```

### 起動と終わり

1. 設定を読み、手順 1（初期化。原点復帰を含む）→ PCA9685 に接続して確認（`buttons.installed: true` のとき。全 ch が待機角へ動く。**ペダルは原点復帰だけで踏まない**）。
   成功で `result`（`job_id: "init"`・exit_code 0）→ `state phase=idle`。失敗は `result`（exit_code 4・設定なら 2）→ 本物の shutdown → 終了。
2. 待機中は 1 s ごとに見張る（下記）。コマンドを受けたらジョブを走らせ、終われば `result` → `state phase=idle` へ戻る。
3. `quit`・SIGINT・SIGTERM・標準入力の EOF・見張りの異常・ハード異常のジョブの終わりで、**本物の shutdown（原点復帰 → サーボ OFF → 切断）して終了**する。

### コマンド（標準入力の JSON 1 行）

ジョブのコマンドは `job_id`（空でない文字列）が要る。`result` までのイベント（`state`・`sample`・`csv`・`button`・`result` …）に同じ `job_id` が付く（`events.set_job_id`）。
走っている間に別のジョブのコマンドが来たら `ack ok=false`（「実行中です」）。初期化前・`needs_init` の間はジョブのコマンドは `ack ok=false`（「初期化が必要です」）。

| コマンド | 項目 | 内容 |
|---|---|---|
| `mode` | `job_id`・`mode`（名前か ID）・`plan_in`（パスか null）・`limit_s`（秒か null）・`no_pre_check`（既定 false）・`home_at_end`（既定 true） | 手順 24,25（`--plan-in` と同じ差し替え・手順 24 の前のブレーキ ON も同じ。`plan_in` が null ならジョブの中で計画を作る） |
| `schedule` | `job_id`・`schedule`（`schedule_to_dict` の形）・`no_pre_check`・`home_at_end` | タイムスケジュールの開ループ再生 |
| `steps` | `job_id`・`steps`（例 `"2,25"`）・`home_at_end` | 学習 2・12〜14、IGN-OFF 25 など（`--steps` と同じ検査。**手順 1 は不可**。初期化は `init`） |
| `interactive` | `job_id`・`purpose`（`manual`/`calib`。既定 manual） | 対話ジョブ。間は `jog`/`move`/`home`/`status`/`save_min`/`save_max`（下の「対話ジョブ」と同じ形）を受け、**`quit_interactive`** で終えて待機へ戻る（ランナーは終わらない）。終わりは原点復帰だけ |
| `init` | `job_id` | 手順 1 をやり直す（つないでいれば本物の終了処理をしてから、原点復帰を含む初期化＋ PCA9685 の確認）。失敗したらランナーを閉じる |
| `stop` | （`job_id` 任意） | 実行中のジョブの StopController に中断を要求（SIGINT と同じ停止の流れ。走行するジョブは停車 → N → IGN-OFF → 原点復帰）。終わったら待機へ戻る（`result` exit_code 130）。ジョブがなければ `ack` だけ |
| `quit` | （`job_id` 任意） | 原点復帰 → サーボ OFF → 切断 → 終了（終了コード 0）。ジョブ実行中ならそのジョブを止めてから |

### ジョブの終わり（`home_at_end`）

core の `run_steps` は終わりに必ず shutdown を呼ぶ。`patches.py` の差し替え 4 が `patches.set_job_end("full"|"home"|"keep")` で中身を切り替える。

| `set_job_end` | 終わりにすること | 使うとき |
|---|---|---|
| `full`（既定） | core のまま（原点復帰 → サーボ OFF → 切断） | `--serve` 以外すべて・`quit`・ランナーの終了・`init` のやり直し |
| `home` | 両軸の原点復帰だけ（サーボ ON・接続のまま） | `home_at_end: true`（全体のジョブの終わり）・対話ジョブ |
| `keep` | 何もしない（停車保持のまま） | `home_at_end: false`（シーケンスの段の間） |

`home`・`keep` では `state phase=job_end`（`mode`）を出す（`full` は従来どおり `phase=shutdown`）。非常停止の流れ（`_emergency_before_shutdown`）と、
`brake_kept`／`buttons_unavailable`（ブレーキを保ったまま終える）の扱いは、どのモードでも先に今までどおり走る。後者はつないだ接続を閉じるので、
`result` と `state phase=idle` に `needs_init: true` が付き、次は `init` が要る。

`result` の項目: `job_id`・`exit_code`・`meaning`・`error`・`elapsed_s`・`needs_init`・`runner_exit`（このジョブの後にランナーを閉じるか）。

### 終わった後の判定（A-D1。`serve.decide_after_job`）

| 条件（上から順） | 結果 |
|---|---|
| 非常停止の診断にハード異常（ブレーキ軸・アクセル軸の異常・CAN を読めない） | ランナーを閉じる |
| ブレーキを保ったまま終えた | 待機（`needs_init`） |
| `stop` コマンドで中断 | 待機 |
| 終了コード 0・2・3・6・7・8 | 待機 |
| 終了コード 4・5 で接続が閉じている／例外の文にハード語（通信・アラーム・過電流・電流・Modbus・CAN・接続・ボタンサーボ・PCA9685…） | 閉じる |
| 終了コード 4（上に当たらない） | 閉じる |
| 終了コード 5（上に当たらない。車速が出ない・ボタン関係以外の DriveError など） | 待機 |
| 終了コード 130・143（SIGINT・SIGTERM で中断） | 閉じる |
| 未知の終了コード | 閉じる（安全側） |

SIGINT・SIGTERM は、走行中のジョブがあれば今どおり中断（非常停止の流れ）し、`result`（130/143）の後に本物の shutdown して終了。待機中ならすぐ shutdown して終了。

### 待機中の見張り（A-D3。`serve.IdleWatcher`）

ジョブを走らせていない間、1 s ごとに両軸の `is_alarm_active()`・`read_monitor()`（電流・位置）と CAN 車速を読む（ジョブとは同時に読まない）。
アラーム・過電流（`settings.toml` の `safety.overcurrent_limit_ma`）・通信失敗 3 回連続・CAN 3 回連続で読めない・指令位置 > 0 で電流 0 mA が 3 回連続
（その軸が一度でも電流 > 0 を返した後だけ）のどれかで `watch`（`level=error`）→ ボタンは押さず、ブレーキ軸が正常なら停車保持の開度まで踏み（今の位置より浅くしない）・アクセルは原点へ（`emergency_stop.hold_and_keep`）→ サーボ ON のまま接続を閉じる（`brake_kept` と同じ扱い。原点復帰・サーボ OFF はしない）→ `result`（終了コード 5・`runner_exit: true`）。ユーザー決定 2026-11-20。
1〜2 回目の通信失敗は `watch`（`level=warn`）だけ。

## 対話ジョブ（`--interactive`）

標準入力に JSON 1 行ずつ。どのコマンドにも `id`（任意）を付けると `ack`・`status` に同じ値が返る。
`quit` か標準入力の EOF で終了処理へ（SIGINT・SIGTERM でも終了処理は走る）。

| コマンド | 例 | 内容 |
|---|---|---|
| `jog` | `{"cmd":"jog","axis":"accel","amount_pct":2.0}` | 今の実開度から ±量 [%]（1 回 100 以内） |
| `move` | `{"cmd":"move","axis":"brake","target_pct":20}` | 目標の開度 [%] へ |
| `home` | `{"cmd":"home"}` | 両軸を原点へ |
| `status` | `{"cmd":"status"}` | `status` イベントで位置・開度・車速 |
| `save_min` | `{"cmd":"save_min","axis":"accel"}`（`both` も可） | 今の実位置を `vehicle.min_*_opening_pct` として車両 yaml へ（コメントは保つ） |
| `save_max` | `{"cmd":"save_max","axis":"accel"}` | 今の実位置を `vehicle.max_*_opening_pct` として車両 yaml へ（min+1% 以上・99% 以下。段11） |
| `quit` | `{"cmd":"quit"}` | 終了 |

開度の定義は core と同じ（原点 0%・9500 pulse = 100%）。開度は 0〜`vehicle.max_*_opening_pct` に収める
（超えたら max にして ack に書く）。`jog`・`move` に `"calib":true` を付けると（キャリブレーション画面だけ）上限を
ストロークの端 100% の手前 99%（`CALIB_STROKE_LIMIT_PCT`）まで広げる（max を測るため）。片足ルール（片方が min 超なら他方は min）: 踏む側を動かす前に、
もう一方が min を超えていれば**先に min へ戻す**（ack に書く。戻すのは安全側）。移動速度は
core の走行前チェックと同じ 20 mm/s。

## 差し替え点（`patches.py`。core の挙動を変える所は全部ここ）

| # | 差し替え | 目的 |
|---|---|---|
| 1 | `SessionLog` → `StreamingSessionLog`（`src.robot.core.*` のうち `SessionLog` を名前で取り込んでいるモジュールすべて。今は main・pattern_drive・pre_drive_check） | sample / csv イベント |
| 2 | `main.write_mode_report` → 何もしない | RPD A-2: モード走行のレポート（md・図）は作らない（ログは DB のログ画面） |
| 3 | `hardware.InitReport.add / skip` を包む | init_step イベント |
| 4 | `main.pre_drive_check` / `main.shutdown` を包む | state イベント・走行前チェックを中断可能に |
| 5 | `button_steps.press_button` を包む（ペダル値の同期は**スタブのときだけ**。手順 24 の Ready 前のブレーキ ON は press_button では行わない＝下の差し替え 5b） | 手順 24 のスタブで Ready に入れない件（下記） |
| 11 | `build_hardware`（`core.hardware`・`core.main`・`quiet_connect` の各モジュール属性）を包む（**スタブのときだけ**。実機は元の関数の結果そのまま） | スタブ車両を「停止・P（ボタンあり）／0 km/h（ボタンなし）」から始める。core は D クリープ 5 km/h 始まりで、走行前チェックが動かさない決定（2026-11-19）後は車速確認が通らないため |

| 7 | `button_steps.ign_off` → src 版（`patches.ign_off`） | 手順 25: ブレーキ ON → スタートを `buttons.ign_off_press_s`（既定 10 s）押す。停車確認（`no_speed_kmh` 以下）は core と同じ。この値は src 専用キー（`config_store.SRC_ONLY_KEYS`。core の検証に渡す前に合成 yaml から取り除き、`cfg.src_only` で渡す）。ユーザー決定 2026-11-15 |
| 5b | 手順 24 の `Step.handler` を包む（`patches.brake_on_before_step24`。`run.py` が `wrap_step` の前に足す。ボタンありのときだけ。実機・スタブ共通） | handler の前（モードの時刻 0 秒より前）にブレーキ ON を踏み終える。press_button 内だと踏む 0〜0.5 s がモード時刻に入り Ready が約 1.8 s に遅れるため（ユーザー決定）。スタブは有効化＋車速 1 回読みも足す。wrap_step の内側なので中断できる |
| 8 | `button_steps.press_button` を最も外側で包む・`ensure_buttons_connected`（core_hardware・button_steps・schedule_job の名前の取り込みも）→ ボタンなしのとき何もしない | D-15c（ユーザー決定 2026-11-16）: 車両 yaml の src 専用キー `buttons.installed: false`（SG90 を外している）のとき、PCA9685 に繋がずサーボを動かさず、`ボタンなし: 「…」を押したことにします` をログに残すだけ（押す秒数・押した後の待ちは同じだけ待つ。ブレーキ ON などの前提動作は元のまま）。設定は `patches.configure(cfg)` がランナーの起動時に渡す。段11 でボタンの設定（`buttons:` 全部）は共通設定へ移した（車両 yaml に `buttons:` が残っていても無視する） |
| 6 | 学習の手順（2・12〜14）の `Step.handler` を包む（`learning_buttons.py`。`run.py` が `wrap_step` の後に足す） | D-14b: tests の学習運転にはボタン操作が無いので、src 側で手順 24 と同じ Ready・D・N を足す（下記） |
| 10 | `_release_pedals`（core の pattern_drive・mode_drive・excite と src の schedule_job の属性）・`button_steps.press_shift_n_after_drive`・`main.shutdown`（4）を包む（`install_emergency_stop`） | 走行するジョブ（手順 2・12〜14・24・スケジュール。対話・ボタンだけのジョブは対象外）の非常停止を `emergency_stop.py` の流れにする。きっかけは SIGINT（shutdown の前に流れを走らせる。走行前チェックが済んでいるときだけ）と自動の非常停止（DriveError。`_release_pedals` の中で走らせ、理由は今の例外の文言）。流れは `asyncio.shield` した 1 本の task で、後から来る SIGINT は同じ流れを待つだけ。流れが済んだら N の二重押しをしない。止まらないとき（`brake_kept`）と、停車しているのに N を押せないとき（`buttons_unavailable`。ブレーキは停車保持の開度で止めたまま）は shutdown で原点復帰・サーボOFF を飛ばす。秒数: スタート `buttons.ign_off_press_s`（10 s）・CAN を読めないときの踏み込み `emergency.can_lost_press_s`（src 専用キー、既定 20 s）。2026-11-19 |
| 9 | `button_steps.press_button` を最も外側で包む（8 の後。`install_button_events`） | 段11: ボタンの押し始め・`seconds` 後の離しを `button` イベントにする（ペダルの図の矩形。ボタンなしでも出る） |

**学習の手順のボタン操作（段7b、D-14b）**: 走行前チェックの後、最初の学習の手順の前に「ブレーキを停車保持の開度（`stop_brake_opening_pct`。新しい車の仮値でもそのまま）に → スタート短押しで Ready →
Ready から `buttons.d_press_at_s` 後に D（待つ間は車速を読み続ける）→ `after_press_wait_s` 待つ」、最後の学習の手順の後に `press_shift_n_after_drive`（ブレーキ ON で N）。
`1,13,14` のように続くときは Ready・D・N とも 1 回。学習の手順が例外で終わったときも N を押す（Ready を押し始めていた場合。N の失敗は元のエラーを優先）。
SIGINT / SIGTERM の中断では N を押さず shutdown を優先（手順 24 と同じ）。D に入ったかの検出は足さない（学習にはモードの基準車速が無い。core の手順 2 の判定に任せる）。
学習の手順と手順 23・24 は同じ実行にできない（`check_job`。Ready でスタートを 2 回押すと IGN-OFF）。押す関数は core の `button_steps`（スタブでは差し替え 5 が効く）。

加えて、core を直さずに次のことをしている（差し替えではなく使い方）:

- 各 `Step` の `handler` / `prepare` を `dataclasses.replace` で包む（state イベント・中断・最後のエラー文の記録）。
- `cfg.save` を `config_store.install_save_sync` で包む（手順 2・12〜14 の保存の直後に車両 yaml へ写す）。
- `--mode`: `cfg.modes.wltp_mode_name` と合成 yaml の同じキーを書き換える（core は手順 3・24 のモードをここから引く）。

## 中断

SIGINT・SIGTERM を受けたら、実行中の**手順の本体だけ**を cancel して `Interrupted` を投げる。
`run_steps` の except 節は拾わないので finally（走行ログのサンプラー停止 → 原点復帰 → サーボOFF → 切断 →
ログ保存）にそのまま進み、終了処理中は cancel されない。2 回目以降のシグナルは無視する。
通信の応答待ち中の cancel は握りつぶされることがあるため、手順の本体が終わるまで 0.5 s ごとに cancel を
繰り返す。終了コードは 130（SIGINT）/ 143（SIGTERM）。

## 取り込み直し

tests/research を直したら `scripts/port_research.py` を再実行 → 等価確認①
（`scripts/check_config_equivalence.py`）・②（`scripts/check_runner_equivalence.py`）を回す。
`patches.py` が差し替える名前（`main.write_mode_report`・`main.pre_drive_check`・`main.shutdown`・
`InitReport.add/skip`・`SessionLog`・`_release_pedals`・`press_shift_n_after_drive`）が core に残っているかは `tests/unit/test_robot_patches.py` が確かめる。

## Web 側（段3・段4）: JobManager・車両 API

Web サーバー（`src/web/`）は**常駐ランナー（`--serve`）を 1 本** `src/app/job_manager.py` の `JobManager` から起動し、
初期化後はサーボ ON・接続のまま、ジョブは標準入力のコマンドで送る（段3・2026-11-20。ジョブごとにランナーを起動しない）。
ジョブの区切りはイベントの `job_id`（`result` が終わり）。同時に 1 ジョブで、走っている間の別の開始は 409。
`initialized` ＝ 常駐ランナーが生きていて初期化済み（最後の idle / result に `needs_init` なし）。
`stop`（走行終了）はコマンド `stop`（非常停止の状態にならない）、非常停止（画面・スイッチ・AC 断）は SIGINT（ランナーは停止の流れ →
終了処理 → 終了。その後は初期化から）、アプリ終了は `quit`。待機中の見張り `watch`（warn/error）は WARNING/ERROR のメッセージにし、
error でランナーが終わると非常停止の状態（きっかけ auto）にする。シーケンスは最後の run だけ `home_at_end: true`（段の間は停車保持のまま）。HW の種類は環境変数から決める: `DRIVING_ROBOT_USE_REAL_HW=1` なら `--hw real`、
それ以外は `stub`（ベンチ `DRIVING_ROBOT_BENCH_GPIO_ONLY=1` も stub。GPIO・UPS だけ実機）。
車両フォルダ・共通設定の場所は `DRIVING_ROBOT_VEHICLES_DIR`・`DRIVING_ROBOT_COMMON_CONFIG`（既定 `vehicles/`・`config/robot_common.yaml`）。

| ジョブの種類 | 手順 | run_type（セッション） |
|---|---|---|
| `learning` | D-15c: full ＝ 2,25 / fel ＝ 12,25 / hold ＝ 13,25 / stop ＝ 14,25（手順 1 は初期化ページだけ。21・22 は外した）。**初期化済みでないと 409**。Ready・D・N は学習の手順にランナーが足す（`learning_buttons.py`） | learning |
| `mode_drive` | 常に 24,25（走行前チェック → Ready・D・モード・N → IGN-OFF。手順 1・21・22 は含まない）。**初期化済みでないと 409**。`--mode` はモードの名前 | auto |
| `ign_off` | 25（常駐ランナーへ `steps`。初期化済みでなければ先に初期化を走らせる。IGN-OFF だけ。走行・学習がエラー・中断で終わって車が Ready のまま残ったときの後始末） | **作らない** |
| `schedule` | （手順 1 は入れない。初期化済みでなければ先に初期化を走らせる）→ 走行前チェック → スケジュール再生（`schedule` コマンド。手順番号 30） | auto（ref なし） |
| `interactive`（手動 `manual`） | `interactive` コマンド（終わりは `quit_interactive`。ランナーは残る） | manual |
| `interactive`（キャリブレーション `calib`） | 同上（min 測定のジョグ） | **作らない** |
| `init_only` | 1（初期化の確認用） | **作らない** |

**セッションの判断**: 学習・モード走行・手動運転だけ `drive_sessions` を作る（vehicle_id＝車のフォルダ名・profile_id は NULL）。
キャリブレーションと init_only は走行ではなく残す値が無いので作らない（進行のメッセージは画面のログバッファにだけ残る）。
手動運転はセッション・メッセージは残るが drive_logs は無い（対話ジョブは CSV を出さない）。
status: 終了コード 0 → completed、非常停止で止めた → emergency、それ以外 → error（利用者が止めた学習・走行の 130 も error）。
例外: 対話ジョブは利用者が閉じる（quit_interactive）のが通常の終わりなので 0・130・143 を completed とする。

**状態**: 待機（standby。画面の `robot_state` は READY）/ 実行中（running。初期化中は INITIALIZING・走行は RUNNING・
手動は MANUAL・キャリブレーションは CALIBRATING）/ 非常停止中（emergency。解除まで次のジョブを受け付けない）。

**CSV の取り込み**（ジョブの終わり。異常終了も）: `csv` イベントの CSV を drive_logs へ入れる。
core の `CSV_COLUMNS`（83 列）のうち次の 9 列は専用の列へ、残りの全列（空欄を除く）は `detail JSONB`（キー＝列名・値＝CSV の文字列）。

| drive_logs の列 | CSV の列 |
|---|---|
| timestamp | timestamp（CSV は JST の naive → DB は UTC） |
| ref_speed_kmh / actual_speed_kmh | 同名 |
| accel_opening / brake_opening | accel_cmd_pct / brake_cmd_pct（**指令**開度。実開度は detail の accel_actual_pct） |
| accel_pos / brake_pos | accel_cmd_mm / brake_cmd_mm × 100（pulse） |
| accel_current / brake_current | 同名 |

NOT NULL の列が空欄の行は 0 にする。`kind=drive` の CSV（と隣の `.yaml`）は取り込み後に消す。`kind=learning`
（`vehicles/<車ID>/data/`）は学習データなので残す。DB が無い環境では取り込まず CSV は消さない。
`/sessions/{id}/logs.csv` は逆向きに detail を展開し、core の `CSV_COLUMNS` の順で全列を書き出す
（tests の CSV と同じ列なので research-run-analysis の道具が読める）。

**停止・非常停止**: どちらもランナーへ SIGINT（終了処理＝原点復帰まで済ませて終わる）。`KILL_TIMEOUT_S`（40 s。core の
原点復帰のタイムアウト 30 s ＋余裕）で終わらなければ SIGKILL し、その旨を進行のメッセージに残す。非常停止は GPIO
（実機・ベンチのみ。`GPIOMonitor`）・AC 断（UPS）・`POST /drive/emergency` から同じ `JobManager.emergency()` に入る。
解除は `reset-emergency`（ランナーが終了処理中は 409・物理スイッチが押されたままなら 423）。アプリ終了時も走っているジョブを SIGINT で止めて待つ。

### Web API（段3・段4）

`/api/v1/drive/*`（画面が使っていたパスを、新しい意味で残した）:

| 区分 | パス | 内容 |
|---|---|---|
| 残した | `GET /status` | `robot_state`・`active_profile_id`（選んでいる車）・`manager_state`・`job`・`last_job`・`prepare`（段4。`{id, kind, name, state: planning｜ready｜waiting｜error, done, total, message}` か null。WebSocket にも同じ） |
| 残した | `GET /log` | サーバーログ＋ランナーの進行のメッセージ（`LogBufferHandler` の同じ形） |
| 残した（意味変更） | `POST /initialize` | init_only ジョブ（手順 1） |
| 残した（意味変更） | `POST /start {mode_id, limit_s?}` | mode_drive ジョブ。`limit_s` は開発用（ランナーの `--limit-s`） |
| 残した（意味変更） | `POST /stop` | 実行中のジョブへ SIGINT（無ければ `{"status":"idle"}`） |
| 残した | `POST /emergency`・`/reset-emergency` | 非常停止・解除（423 スイッチ押下中・409 終了処理中） |
| 残した（意味変更） | `POST /manual/start・jog・home・stop` | 手動運転（対話ジョブ）。jog の `step` は pulse のまま（% へ換算） |
| 残した（意味変更） | `POST /calib/jog・home` | キャリブレーション（対話ジョブ。最初の呼び出しで始まる）。終わりは `/stop` |
| 残した（意味変更） | `POST /learning-cycle/start・abort`・`GET /learning-cycle/status` | 学習ジョブ（body 省略は full）・中断・状態 |
| 残した（互換） | `POST /arm`・`/cancel`・`/learning-cycle/arm`・`/learning-cycle/cancel` | 旧「準備」の確認ポップアップ用。arm は始められるかの確認だけ・cancel は何もしない（段5 で画面から外す） |
| 残した（意味変更） | `POST /select-profile` | 走らせる車を選ぶ（`vehicles/.active`。ジョブ中は 409） |
| 足した | `POST /learning/start {kind: full｜fel｜hold｜stop}` | 学習ジョブ |
| 足した（D-15b） | `POST /ign-off` | IGN-OFF だけのジョブ（25）。`last_job.needs_ign_off`（手順 21・22・24 を始めたのに 25 が正常に終わっていない学習・走行ジョブ）のとき画面が「車は Ready のまま」の注意とこのボタンを出す |
| 足した | `POST /calib/save-min {axis: accel｜brake｜both}` | 今の位置を min として車両 yaml へ保存し「測定済み」にする |
| 足した（段6） | `POST /schedule/start {schedule_id}`・`/schedule/stop` | タイムスケジュールの再生ジョブ（下の「段6」）。検査に通らなければ 422 |
| 足した（段6） | `POST /sequence/start {sequence_id, limit_s?}`・`/sequence/stop` | シーケンス（段を順に 1 ジョブずつ）。`limit_s` はモードの段だけ（開発用） |
| 足した（段4） | `POST /prepare {kind: mode｜schedule｜sequence, id, limit_s?}` | 走行の準備: 違うモードごとに計画プロセス（`--plan-only`）を 1 本ずつ順に走らせる（`vehicles/<車ID>/.work/plans/<準備ID>/`）。応答 `{prepare_id, state: planning｜ready, done, total}`。準備中・ジョブ中・非常停止中・未初期化は 409、参照先なしは 404。準備中は他の開始も 409 |
| 足した（段4） | `POST /prepare/cancel` | 計画プロセスを止め（SIGINT → 5 秒で SIGKILL）ファイルを消す。応答 `{status: cancelled}`（無くても 200）。非常停止・アプリ終了でも同じ |
| 足した（段4） | `POST /prepare/go` | 「はい」。できていればジョブを始める（`{state: started, job_id}`。モードのランナーに `--plan-in`、シーケンスの 2 段目以降に `--no-pre-check`）。まだなら `{state: waiting, job_id: null}`（できたら自動で始める。失敗なら state=error）。終わったら計画ファイルを消す |
| 外した | `/calibrate`・`/calib/set-zero`・`/calib/set-full`・`/calib/save` | ゼロ/フル点は廃止（D-13） |
| 足した（段11） | `POST /calib/save-max {axis: accel｜brake}` | 今の位置を max として車両 yaml へ保存（min+1% 以上・99% 以下。`/calib/jog` は `calib:true` で 99% まで動かせる） |
| 足した（段11） | `POST /pause`・`/resume` | モード走行ジョブのランナーへ SIGUSR1 / SIGUSR2（job の `paused` を持つ）。モード走行以外・状態違いは 409（日本語）。**ランナー側の一時停止は後で実装**（それまでは受けて「未対応」と言うだけ） |
| 外した | `/plan/*`・`/pid-tune/*`・`/learning/train`・`/learning/arm|cancel`・`/learning-cycle/…`の旧 body | プラン・PID 適合・旧学習サイクルは廃止 |

`/api/v1/profiles/*`（車両 yaml。ID は車のフォルダ名）:

| メソッド・パス | 内容 |
|---|---|
| `GET /` ・`GET /{id}` | 車の一覧・1 台。`ProfileResponse`: 諸元・min/max・`min_*_measured`（false ＝ 仮値）・`buttons`・`learned`・`model_path`・`learned_at`・`candidate`・`feedforward`（学習の値。読むだけ） |
| `POST /` | 共通の雛形から作る（`name` 必須・`id` 省略可・諸元の仮値）。学習の値なし・min は未測定。クリープ加速カーブは core の検証のため仮の 2 点（手順 2-0 が置き換える） |
| `PUT /{id}` | 人が決める値（`vehicle.*`）だけ。送ったキーだけ書く。core の `validate_config` を通らなければ 422（日本語の理由）。知らないキー（feedforward・buttons など）は 422（ボタンの設定は段11 で共通設定へ移した）。min を書いた軸は測定済みになる |
| `DELETE /{id}` | フォルダごと消す。この車のジョブが走っていれば 409 |

**選んでいる車**: 既存の `select-profile`＋`GET /status` の `active_profile_id` の仕組みのまま、サーバー側（`vehicles/.active`、
再起動後も残る）に持つ。理由: `/initialize`・`/start` に車の指定が無く、LAN の複数の画面で同じ車を見るため。
`app.js` は起動時に `status.active_profile_id` を見て画面の選択を合わせる作りなので、画面側は変えなくてよい。

`GET /api/v1/sessions/`（`vehicle_id`・`vehicle_name` を追加。`profile_id` は NULL 可）・`/sessions/{id}/messages`（進行のメッセージ）・
`/sessions/{id}/logs.csv`（全列）・`/sessions/{id}/analysis`（段11: モード走行のセッションの運転指標 EER・IWR・ASCR と偏差の KPI・図・区間別。`src/web/analysis.py`）。WebSocket `RealtimeData` は `cycle_progress` を `job`（`JobSnapshotSchema`）に置き換え、
電流は常に 0.0（ランナーの sample に無い）。


## 段6: タイムスケジュール再生・シーケンス

**再生（`schedule_job.py`）**: 開ループ。ペダル開度の点を時刻で線形補間し、core の周期（50 ms）で `move_to_position`・
`read_monitor`（core の `AxisSafetyNet`・過電流・最高速・CAN/通信失敗で `DriveError`＝終了コード 5。ペダルを離してから）。
開度は core の定義（原点 0%・9500 pulse = 100%）。`schedule_spec.pedal_command`: 0〜max（`vehicle.max_*`）に頭打ち →
使わないペダルは待機位置（min）。アクセルとブレーキの同時踏みは許す（2026-11-19 ユーザー決定。片足ルール・実開度の安全網は掛けない）。
ボタンは `button_steps.press_button` で指定時刻に背景タスクとして押す（失敗は警告を残して走行は続ける＝旧 `schedule_loop` と同じ）。
押す秒数は `press_duration_s`、無ければ車両 yaml（スタートは `start_short_press_s`・P/N/D は `shift_press_s`）。
終端（段7）: 車が動いていれば core の緩減速（`stop_decel.decelerate_to_stop`＝モード走行の DECEL_TO_STOP と同じ）で停車まで減速して
停車保持し（区間名 `DECEL_TO_STOP`）、止まっていればアクセルを原点・ブレーキを停車保持開度にしてすぐ終わる。その後に終了処理（原点復帰・サーボOFF・切断）。
スタブで 55 km/h から 26 秒で停車した。
記録は `StreamingSessionLog`（区間名 `SCHEDULE_DRIVE`・ref なし。`csv` イベントの kind は drive）。

**ボタンは名前で指す**: `ButtonEvent.button`（`start`/`P`/`N`/`D`）。旧データ（`channel` だけ）は読み込みで `button=None`
（名前なし・要再設定）のまま残し、`channel` も保存し直すまで保つ（DB の `button_events` は JSONB なのでスキーマ変更なし）。
実行前の検査（`schedule_spec.check_schedule`。Web は 422・ランナーは終了コード 2）: 名前なし・知らない名前・
点もイベントも無い・時刻が増えない。

**シーケンス（`JobManager.start_sequence`）**: 全体の繰り返し × 段 × 段の繰り返しを 1 回ずつ展開し（`expand_runs`）、
1 回 = 1 ランナー起動 = 1 セッション（run_type auto）で順に走らせる。mode の段は手順 24。
どれかが 0 以外で終わったら残りは走らせない（state `error`）。停止（`stop`）・非常停止（`emergency`）で全体を止める
（`stopped`/`emergency`）。実行中は別のジョブ・シーケンスを始められない（409）。進み具合（`cycle/cycles`・
`step_no/steps_total`・`step_repeat/step_repeats`・`run_no/runs_total`・結果の配列）は `GET /drive/status` の `sequence`/`last_sequence` と
ジョブの `job.sequence`、WebSocket の `sequence`（段の切れ目は `job` が null でもここは残る）。

**モード走行は常に手順 24（2026-11-15 ユーザー決定）**: `buttons.enabled`・手順 3 への切り替えは廃止
（`config_store.buttons_enabled`・車両 API・画面の「ボタン操作あり/なし」・ランナーの手順 3 を外した）。スケジュールのボタンイベントも常に有効。

**スタブで手順 24 が「D に入っていない可能性」で止まっていた件（差し替え 5）**: 原因は core のスタブ車両の状態
（`VehicleStatus`）の受け取り方。`ensure_buttons_connected` が状態を有効化する（IGN-OFF・P・ペダル 0 から）が、ペダルの値を
受け取るのは CAN 車速を読んだとき（`StubVehicle.advance`）だけ。手順 24 は最初のボタン操作（Ready のスタート短押し）が
モードの 0 秒で、有効化の直後・車速を読む前（`poll=False`）に押すため、ブレーキ 0 と見なされて「ブレーキを踏んでスタート」が
成立せず IGN-ON(RLY-ON) になり、D に入れず 3 秒で中止（終了コード 5）になっていた（手順 20〜23 は直前に車速を読むので起きない）。
`patches.install_stub_button_sync` が、**スタブのときだけ**ボタンを押す直前に有効化と車速 1 回の読み取りを足す
（実機 `hw.is_real` は元の関数をそのまま呼ぶ。`tests/unit/test_stub_button_sync.py` で確認）。

## 段7: 古い制御の削除・取り込み直しの注意

- 古い制御（`RobotController`・学習サイクル・PID 適合・プラン・`src/domain/*` など）と、それだけが使っていたモジュール・スクリプト・テストを
  削除した。一覧は `docs/Phase/RPD_Phase_20261115_PortResearch.md`「4. 結果」段7。DB の旧テーブル（`vehicle_profiles`・`learning_cycles`・
  `pedal_plans` など）はデータを残すだけで使わない（`scripts/setup_db.py` の作成文も残している）。
- 取り込み直しの後の等価確認②は、tests 側の走行も手順 24 で行い（`tests.research.main --steps 1,24`）、`check_runner_equivalence.py` に渡す
  （ランナーは手順 3 を動かさない）。
- `src/` の外（`tests/research`）は変えない。tests/unit の src 用テストは `src.robot.core` を import してよい（`tests` の import だけ禁止。
  `tests/unit/test_no_tests_import.py`）。

## 段7b（D-14b・D-15b）: 学習のボタン操作・通しの流れ

- 学習の手順の Ready・D・N は上の「差し替え 6」。モード走行・学習運転は画面からは毎回**通しの流れ**で走る（モードは `1,24,25`、学習は `1,<学習の手順>,25`）。
  毎回 IGN-OFF で終わるので、続けて走らせても車の状態がずれない（`1,24` を続けると 2 本目の Ready 押しで IGN-OFF になり D に入らない）。
- 学習の手順の準備（読み込み・計画）は core の `preload_target` で手順 21・22 の前に済む（ボタン操作の途中で車を待たせない）。
- `check_job`: 学習の手順と手順 23・24 は同じ実行にできない（21・22・25 は許す）。
- 中断・非常停止では手順 25 も N も押さない（shutdown 優先）。エラーで終わったときは core の `run_steps` が後の手順へ進まないので手順 25 も押さない
  （学習は N だけ押す）。車は Ready のまま残るので、JobManager が `needs_ign_off` を付け、画面が注意と IGN-OFF ボタン（`POST /ign-off`＝`1,25`）を出す。

## 段9（D-15c、2026-11-16）: 初期化ページだけで初期化・ボタンなし

- **手順 1 を行うのは初期化ページのジョブ（`init_only`）だけ**。他のジョブ（手順の並びに 1 が無いもの）は、ランナーが `run_steps` の前に
  `quiet_connect.prepare_hardware` で `ctx.hardware` を作り、アクセル/ブレーキ/CAN を開く → Modbus 操作権 → アラームが出ていればリセット →
  サーボ ON（`checks.init_ups` が true なら UPS のポーリング開始）だけを静かに行って `initialized` にする（core の `require_hardware` を通すため）。
  原点復帰はしない（直前のジョブの終了処理が済ませている。原点にあるかは走行前チェックの「アクチュエータ位置」が確かめる）。
  失敗は終了コード 4・中断は 130/143（どちらも開いた接続を `shutdown` で閉じる）。対話・スケジュールのジョブは従来どおり手順 1 を自分で行う。
- **JobManager の関門**: 初期化ジョブが正常終了すると `initialized` が true。非常停止（`emergency()`）・正常でない終了（0 以外。利用者の停止の 130 も含む）・サーバー起動で false。
  false の間、学習・モード走行（シーケンスのモードの段を含む）は 409（`NOT_INITIALIZED_MESSAGE`）。IGN-OFF・対話（手動・キャリブレーション）・スケジュールは関門なし。
  `GET /drive/status` と WebSocket に `initialized`、`/drive/status` に機器の有無 `devices`（ups・gpio・buttons）。
- `buttons.installed`（src 専用・既定 true・プロファイル画面から読み書き可）と `[web_gpio] enabled`（settings.toml。tests/research が `[gpio]` の未知キーを拒否するため別の節。
  環境変数 `DRIVING_ROBOT_GPIO_DISABLED=1` でも切れる。実機モードだけで効く）。off のとき GPIO の見張りを作らない＝`emergency_probe` なし（解除が 423 にならない）。
