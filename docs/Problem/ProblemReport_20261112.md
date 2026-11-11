# 20261112 スタートスイッチ・シフトレンジのボタンを押す手順（手順20〜25）を tests に足す

**状態: 進行中（段0〜5 完了＝机上まで・段6 は実機待ち）**

研究ハーネス（`tests.research.main`）から、PCA9685＋SG90 でスタートスイッチと P/N/D ボタンを押し、
IGN-ON → メンテナンスモード → N レンジホールド → Ready → モード走行 → IGN-OFF を無人で回せるようにする。
SG90・PCA9685 はまだラズパイに付いていないので、今回は机上（単体テスト・スタブ）まで。

## 問題

- 今の手順3（モード走行）は、車が Ready・D レンジになっている前提で始まる。IGN-ON・Ready・レンジの切り替えは人が手で行っている
- 手順をまとめて無人で回すには、スタートスイッチとレンジボタンをロボットが押す必要がある
- 押す流れ（メンテナンスモードの入り方など）は車種によって違う。試験車の流れをコードに固定すると他の車で使えない

## 議論で確認した事実

| 項目 | 内容 |
|---|---|
| 既存のボタンサーボドライバ | `src/infra/button_servo_driver.py`（`ButtonServoDriver`）。smbus2 で PCA9685 を直接制御する。`press(ch, 秒)` は finally で必ず待機角に戻す。`release_all()` で全チャンネルを待機角へ。ch0＝スタート・ch1〜3＝P/N/D は今回の割り当てと同じ |
| 動作確認スクリプト | `tests/hardware/test_servo_pca9685.py`（サーボ 1 個を手で動かす。配線メモつき） |
| 角度 | 待機 60°・押下 110°（`ServoSettings` の既定値。**実機では未確認**）。`tests/research/app_settings.py` に `ServoSettings` が移植済み（`config/settings.toml` の `[servo]` を読む。今は節が無いので既定値） |
| smbus2 | `.venv` に 0.4.3 が入っている |
| src と tests の分離 | `tests/research` は src を import しない。ドライバは `tests/research` に持ち込む |
| 手順の実行順 | `run_steps` は `--steps` に書いた順に実行する。走行前チェックは `drives=True` の最初の手順の前に 1 回だけ行う |
| 走行前チェックの後 | ブレーキは停車保持の開度（`stop_brake_opening_pct`）で保持している。手順3 はこの状態からモードを始める |
| 状態の読み取り | 車速（CAN）以外の状態（Ready か・今のレンジ）は読めない。ボタンを押しても結果は確認できない（開ループ） |
| CAN と IGN-OFF | 手順1 の CAN 通信チェックは今までどおり行う。実車では IGN-OFF の間に CAN が止まることがある（推測。シミュレータは送り続ける） |
| シミュレータの仕様 | アップロードされたアイデア資料（車両シミュレータの状態）。押した瞬間の遷移（表1）、押し続けたときの遷移（表2：2 秒で RLY-OFF・5 秒で IGN-OFF）、レンジの条件（P＝停車中・N/D＝ブレーキ ON）、N ホールド（IGN-ON で P から N を 5 秒以内に 2 回）、メンテナンスモード（RLY-OFF から 60 秒以内に P×3・N×3・P×3 のアクセル踏み） |
| N ホールドの条件と順番 | 成立は「IGN-ON・P から」。最初に考えた順番（Ready・モード走行の後）では、Ready・N なので成立しない |

## 検討した案

| 案 | 内容 | 判断 |
|---|---|---|
| 流れも yaml の操作列にする | 車ごとに操作列を書き換える | 不採用（今回は過剰。回数・時間・開度を yaml にすれば足りる） |
| **流れはコード・値は yaml（採用）** | 押すボタンの順はコードに書き、回数・押す秒数・開度・待ち時間は yaml の `buttons:` 節に置く | 採用 |
| N ホールドをモード走行の後に入れる | Ready・N から P に入れて N を 2 回押す | 不採用（資料の条件は IGN-ON。Ready では成立しない） |
| **N ホールドをメンテナンスモードの後に入れる（採用）** | IGN-ON(RLY-OFF)・P の間に入れる。Ready になっても続き、IGN-OFF の後も N のまま | 採用 |
| **手順21 の先頭で手順20 も行う（採用）** | RLY-OFF から 60 秒以内の制約を手順の中で守る | 採用 |
| 手順1 の CAN チェックを外せるようにする | IGN-OFF で CAN が止まる車への対応 | 不採用（今までどおり行う） |
| D を押した後にブレーキを緩めて動き出しを見る | 走る前に D に入ったかを確かめる | 不採用（余計な操作はしない） |
| **モード中に車速が出なければ中止（採用）** | 基準車速が 0 を超えてから一定時間、実車速が出なければ中止 | 採用 |
| **スタブに車両の状態を持たせる（採用）** | アイデア資料の状態遷移をスタブで実装し、机上で流れの最後の状態を確かめる | 採用 |

## ユーザー決定（2026-11-12）

- PCA9685 の ch0＝スタートスイッチ・ch1＝P・ch2＝N・ch3＝D
- 手順は次の番号にする（番号を振り直す案 1）
  - 20 IGN-ON(RLY-OFF)
  - 21 メンテナンスモード（先頭で 20 を行う）
  - 22 N レンジホールド
  - 23 Ready
  - 24 モード走行（停車保持の開度で D → モード → 終わったら N）
  - 25 IGN-OFF
- 押す流れはコード、回数・秒数・開度は yaml（論点1 案 1）
- 手順1 の CAN 通信チェックは今までどおり
- 手順24 は D を押したらそのままモードを走り、車速が出なければその時点で中止する
- 「車速が出ない」＝基準車速が 0 km/h を超えてから 3 秒間、実車速が 0.2 km/h 以下のまま
- 机上はスタブに車両の状態（アイデア資料の状態遷移）を持たせて確かめる
- 今回は机上まで。実機確認は SG90・PCA9685 を付けた後に、手順を 1 つずつ確かめてから一連の流れで確かめる

## 全体計画

計画の詳細: `/home/raspi5_16gb/.claude/plans/nifty-puzzling-cake.md`
1 段ずつ実装 → Claude が確認（ruff・変更箇所のテスト・机上・スタブ）→ 次の段へ（ユーザー確認で止めない）。実機確認はハードを付けた後。テストは変更箇所のみ。

| 段 | 内容 | 走行 | 状態 |
|---|---|---|---|
| 0 | 本ドキュメント作成 | なし | ✅ 完了 |
| 1 | ボタンサーボのドライバを tests に持ち込む（実機・スタブ）＋ハード層へ組み込む | なし（オフライン） | ✅ 完了（ドライバ・スタブ・hardware 組み込み・Claude 確認 2026-11-12） |
| 2 | yaml に `buttons:` 節を足す | なし（オフライン） | ✅ 完了（ButtonsSection・検証・yaml・Claude 確認 2026-11-12） |
| 3 | スタブの車両の状態（アイデア資料の状態遷移） | なし（オフライン） | ✅ 完了（VehicleStatus・StubVehicle 連携・Claude 確認 2026-11-12） |
| 4 | 手順20・21・22・23・25 | スタブ | ✅ 完了（5 手順・スタブで一連の流れが予定どおり・Claude 確認 2026-11-12） |
| 5 | 手順24（D → モード → N・車速が出なければ中止） | スタブ | ✅ 完了（スタブ 60 s で完走・D 無視で中止して N を押す・Claude 確認 2026-11-12） |
| 6 | 実機確認（1 つずつ → 一連の流れ） | 実機（ハード取り付け後） | ⬜ 未着手（ハード待ち） |

## 各段の内容

### 段1: ボタンサーボのドライバを tests に持ち込む

**内容**

- `tests/research/button_servo.py` を新しく作る。`src/infra/button_servo_driver.py` から持ち込む（src は import しない）
  - 実機：`ButtonServoDriver`（connect / press / release_all / close）
  - スタブ：`StubButtonServo`（押した ch・秒数・時刻を記録。段3 の車両の状態へ押した事実を伝える）
  - チャンネル：`CH_START=0`・`CH_P=1`・`CH_N=2`・`CH_D=3`
- `tests/research/hardware.py` の `ResearchHardware` に `buttons` を足す。角度・I2C は `app_settings.ServoSettings`（`config/settings.toml` の `[servo]`）から読む
  - 実機の PCA9685 は**最初にボタンを押す手順で接続する**（手順1 では接続しない。PCA9685 が無くても手順2・3 は今までどおり走る）
  - 終了処理（`shutdown`）で、接続していれば `release_all()` → `close()`

**結果**（Claude 確認 2026-11-12）

- 新規 `tests/research/button_servo.py`（`ButtonServoDriver`・`StubButtonServo`・`CH_START/P/N/D`・`BUTTON_NAMES`）、`hardware.py` に `buttons`・`ensure_buttons_connected`・`shutdown` の解放/切断を追加
- 新規テスト `test_research_button_servo.py` 13 件＋既存 `test_research_hardware` を合わせて 45 件 全て通過。`ruff check` 通過
- 手順1（`run_initialize`）では PCA9685 に接続しないこと、`shutdown` で接続時だけ `release_all()`→`close()` になることをテストで確認
- 補足: `servo.enabled`（`settings.toml` の既定 false）は見ていない（研究ハーネスはボタンの手順を明示したときだけ使うため）
- 車両の状態（`vehicle_status`）との接続は段3 で行う（段1 では `hw.vehicle_status` は None のまま）

### 段2: yaml に `buttons:` 節を足す

**内容**

- `tests/research/config.py` に `ButtonsSection` を足し、`config_testVehicle.yaml` に `buttons:` 節を書く

**調整値**

| 値 | 現在 | データの根拠 | 何に効くか | いつ決めるか |
|---|---|---|---|---|
| `start_short_press_s` | 0.5 | 仮の値（根拠なし）：表2 の 2 秒より十分短い | Ready・IGN-OFF の押下 | 実機確認（段6） |
| `start_long_press_s` | 3.0 | 半分データ由来：資料の「2 s 以上 5 s 未満」の真ん中 | IGN-ON(RLY-OFF) | 実機確認（段6） |
| `shift_press_s` | 0.5 | 仮の値（根拠なし） | P/N/D の押下 | 実機確認（段6） |
| `after_press_wait_s` | 1.0 | 仮の値（根拠なし）：押した後に車が状態を変えるまでの待ち | 全ボタン | 実機確認（段6） |
| `accel_tap_opening_pct` | 10.0 | 仮の値（根拠なし）：資料の「アクセル 1% 以上」を確実に超える | メンテナンスモードのアクセル踏み | 実機確認（段6） |
| `accel_tap_on_s` / `accel_tap_off_s` | 0.5 / 0.5 | 仮の値（根拠なし） | アクセル踏み 1 回の長さ | 実機確認（段6） |
| `maintenance_tap_count` | 3 | 資料の値（車種で変わる） | メンテナンスモード | 車ごと |
| `n_hold_gap_s` | 1.0 | 半分データ由来：資料の「5 秒以内」より十分短い | N ホールドの 2 回押し | 車ごと |
| `no_speed_abort_s` | 3.0 | 仮の値（ユーザー決定）：発進の遅れより十分長い | 手順24 の中止判定 | 実機確認（段6） |
| `no_speed_kmh` | 0.2 | 資料の「停車中」のしきい値 | 手順24 の中止判定 | 車ごと |

- ブレーキ ON の開度は、新しい値を作らず `stop_brake_opening_pct`（手順2 が書く停車保持の開度）を使う

**結果**（Claude 確認 2026-11-12）

- `config.py` に `ButtonsSection`・`ResearchConfig.buttons`・検証（秒数は正・回数は 1 以上・`start_long_press_s` は 2 以上 5 未満）を追加。`config_testVehicle.yaml` に根拠コメントつきの `buttons:` 節を追加
- `test_research_config` 135 件 全て通過（新規 2 件: 既定値と検証・節が無いときの既定）。`ruff check` 通過

### 段3: スタブの車両の状態

**内容**

- `tests/research/vehicle_status.py` を新しく作る。アイデア資料の状態遷移を実装する
  - 状態（IGN-OFF / IGN-ON(RLY-ON) / IGN-ON(RLY-OFF) / Ready）・レンジ（P/N/D）・N ホールド・メンテナンスモード
  - スタートスイッチの表1（押した瞬間）・表2（押し続けた 2 秒・5 秒）、レンジの条件、IGN-OFF のときのレンジ、アクセル 1 回踏みの数え方
  - ブレーキ・アクセルの開度はスタブ軸の位置から、車速は `StubVehicle` から読む

**結果**（Claude 確認 2026-11-12）

- 新規 `tests/research/vehicle_status.py`（`VehicleStatus`・時刻を引数で受け取る純粋なクラス）。`StubVehicle.advance` が active のときだけ状態を見る（Ready・D は従来どおり／N は惰行の半分＋ブレーキで減速／P・IGN-OFF は 0 km/h）。`build_hardware` のスタブで `StubButtonServo` と `VehicleStatus` をつなぎ、`ensure_buttons_connected` で有効化（IGN-OFF・P・車速 0 から）
- 新規テスト `test_research_vehicle_status.py` 29 件（表1 各行・表2 の 2 s/5 s/走行中の緊急 OFF・レンジの条件・N ホールドの成立と解除・メンテナンスモードの成立/崩れたらやり直し/60 s 超え/4 回目・IGN-OFF のレンジ・active でないときに車速の計算が変わらないこと）全て通過。`test_research_button_servo`・`test_research_hardware` と合わせて 74 件通過。`ruff check` 通過
- 決めたこと（資料に明記がなく、机上の実装で選んだ点）
  - ペダルの値は「ペダルに触れた位置（min）を 0 とした開度」で状態に渡す（シミュレータが読むゲームペダルの値に相当）
  - N ホールドの 2 回目の N は、シフト条件（ブレーキ ON）を見ない（資料は 1 回目だけを条件にしている）
  - メンテナンスモードは P×3・N×3・P×3 の最後の 3 回目を踏み終えた時点で成立とする
- スタブ HW では、`StubVehicle` は Ready・D のときだけ今までどおり車速を計算する。それ以外は車速 0（N で走っている場合は資料どおり減速）。ボタンの手順を使わない手順2・3 の机上が変わらないよう、**車両の状態はボタンの手順を実行したときだけ有効**にする

### 段4: 手順20・21・22・23・25

**内容**（`tests/research/main.py` に手順を足す。押す処理は `tests/research/button_steps.py` に置く）

| 手順 | 中身 |
|---|---|
| 20 IGN-ON(RLY-OFF) | ブレーキを待機位置にして、スタートを `start_long_press_s` 押す |
| 21 メンテナンスモード | 手順20 → P でアクセル踏み × 回数 → ブレーキ ON → N を押す → ブレーキを離す → アクセル踏み × 回数 → P を押す → アクセル踏み × 回数（両ペダルを同時に踏まない） |
| 22 N レンジホールド | ブレーキ ON → N を押す → `n_hold_gap_s` 待つ → N を押す → ブレーキを離す |
| 23 Ready | ブレーキ ON → スタートを `start_short_press_s` 押す → ブレーキを離す |
| 25 IGN-OFF | 停車中（車速 ≤ `no_speed_kmh`）を確かめてから、ブレーキを停車保持の開度にしてスタートを `start_short_press_s` 押す |

- どの手順も手順1（初期化）が先に要る。押した時刻・ch・秒数は画面と走行ログに残す
- 机上：スタブで `--steps 1,21,22,23,25` を流し、各手順の後の状態が予定どおりかを確かめる

**結果**（Claude 確認 2026-11-12）

- 新規 `tests/research/button_steps.py`（`press_button`・`set_brake`・`accel_taps`・手順 20・21・22・23・25 と、手順 24 用の D/N 押下）、`main.py` に手順 20・21・22・23・25 を登録（`--list`・docstring にも 20〜25）。`MAX_STEP` が 14 → 25 になったため、`MAX_STEP == 14` を見ていた `test_research_hold_points`・`test_research_stop_approach_step` の 2 行を 25 に更新
- 新規テスト `test_research_button_steps.py`（手順 20〜25 の連続・21 単独・25 の停車確認・手順 2/3 だけの実行では接続しない・手順 24 の D/N/中止）7 件 通過。`ruff check tests/research` 通過
- 机上（スタブ）`--steps 1,21,22,23,25`: **終了コード 0**。各手順の後に表示した車両の状態は次のとおり

| 手順 | State | Range | NHold | Maint |
|---|---|---|---|---|
| 21 の後（先頭で 20 も行う） | IGN-ON(RLY-OFF) | P | OFF | ON |
| 22 の後 | IGN-ON(RLY-OFF) | N | ON | ON |
| 23 の後 | Ready | N | ON | ON |
| 25 の後 | IGN-OFF | N | ON（資料どおり、解除は次の IGN-ON） | OFF |

- 手順 20 単独の確認は、手順 21 の先頭の表示（IGN-ON(RLY-OFF)・P・NHold OFF・Maint OFF）で行った
- 計画に無かった追加が 1 点ある: 手順 23 までで両ペダルは待機位置（min）にあり、走行前チェックの「アクチュエータ位置」（原点から離れていないこと）が NG になった（スタブで再現）。そのため、ボタンの手順を使ったときだけ、走行前チェックの前に両ペダルを原点へ戻す（`button_steps.return_pedals_to_origin`）。手順 2・3 だけの実行は変わらない

### 段5: 手順24（モード走行）

**内容**

- 手順3 と同じ準備（モード・FF モデルの読み込み）と走行前チェックの後、停車保持の開度のまま D を押してからモードを走る
- モードの途中で、基準車速が 0 km/h を超えてから `no_speed_abort_s` の間、実車速が `no_speed_kmh` 以下のままなら中止する（手順24 のときだけ。手順3 は変えない）
- モードが終わったら（停車保持の状態で）N を押す。**中止したときも**、ブレーキを停車保持の開度にして N を押してから終了処理に進む（D のままペダルを離さない）
- 机上：スタブで `--steps 1,21,22,23,24,25 --limit-s 60` が最後に IGN-OFF・N（N ホールドあり）になること。D を押さない場合（スタブの押下を外す）に中止になること

**結果**（Claude 確認 2026-11-12）

- `mode_drive.py` に `NoSpeedWatch`（基準車速が 0 を超えてから `no_speed_abort_s` の間、実車速が `no_speed_kmh` 以下のままなら中止。一度でも超えたら判定を止める）を追加し、`run_mode_drive`・`_ModeRun` に `no_speed_abort`（既定 None＝何もしない）を足した。`main.py` は手順 3 の中身を `_mode_drive_step` に切り出し（コピーなし）、手順 24 は D を押す・N を押す・中止判定の 3 点だけが違う。中止でもブレーキを停車保持の開度にして N を押してから DriveError を投げる
- テスト: `test_research_mode_drive` に中止判定 3 件（出ない→中止・出た→判定を止める・None→何もしない）通過。手順 24 の D/N/中止の流れは `test_research_button_steps` で確認
- 机上（スタブ）`--steps 1,21,22,23,24,25 --limit-s 60`: **終了コード 0**（244 s。うち走行前の準備に約 2 分）。手順 24 は D を押して Range=D で 60 s 走り（最大 |偏差| 1.95 km/h）、終わりに N を押して Range=N。続く手順 25 で IGN-OFF

| 手順 | State | Range | NHold | Maint |
|---|---|---|---|---|
| 23 の後 | Ready | N | ON | ON |
| 24 の D を押した後 | Ready | D | ON | ON |
| 24 の N を押した後 | Ready | N | ON | ON |
| 25 の後 | IGN-OFF | N | ON | OFF |

- D を押しても入らない場合（スタブの D の押下を無視させて同じ流れを実行）: モード走行 14.1 s で「基準車速が出てから 3s たっても車速が 0.2 km/h 以下のまま（D に入っていない可能性）」と中止 → ブレーキを停車保持にして N を押す → 走行エラー（**終了コード 5**）→ 終了処理
- 手順 2・3 は変わっていない: `no_speed_abort` 既定 None・`active` でないスタブ車両の車速は従来どおり（`test_research_hardware`・`test_research_vehicle_status` 通過）。なお `test_research_mode_drive` には、この作業の前から 42 件の失敗がある（pkl の meta に `hold_pi` が無いという ConfigError。変更を退避した状態でも同じ 42 件で、今回の変更とは無関係）

### 段6: 実機確認（ハード取り付け後）

**実行手順**（Claude が実行。SG90 の位置合わせ・角度の調整はユーザー）

```bash
# 1 つずつ（前の手順を含めて重ねていく）
.venv/bin/python -m tests.research.main --hw real --steps 1,20          # IGN-ON(RLY-OFF)
.venv/bin/python -m tests.research.main --hw real --steps 1,21          # メンテナンスモード
.venv/bin/python -m tests.research.main --hw real --steps 1,21,22       # N ホールド
.venv/bin/python -m tests.research.main --hw real --steps 1,21,22,23    # Ready
.venv/bin/python -m tests.research.main --hw real --steps 1,21,22,23,24,25 --limit-s 60
# 一連の流れ
.venv/bin/python -m tests.research.main --hw real --steps 1,21,22,23,24,25
```

見るもの: シミュレータの表示（State・Range・NHold・Maint）が各手順の後に予定どおりか。手順24 で車速が出るか

## 進捗ログ

| 日付 | 内容 |
|---|---|
| 2026-11-12 | 議論・ユーザー決定・計画承認。段0（本ドキュメント）完了。次は段1 |
| 2026-11-12 | 段1 完了（ドライバ・スタブ・hardware 組み込み、テスト 45 件・ruff OK）。次は段2 |
| 2026-11-12 | 段2 完了（`buttons:` 節・検証、test_research_config 135 件・ruff OK）。次は段3 |
| 2026-11-12 | 段3 完了（`VehicleStatus`・StubVehicle 連携、テスト 29 件＋既存と合わせ 74 件・ruff OK）。次は段4 |
| 2026-11-12 | 段4 完了（手順 20・21・22・23・25、スタブ `--steps 1,21,22,23,25` 終了コード 0・状態は予定どおり、button_steps テスト 7 件）。次は段5 |
| 2026-11-12 | 段5 完了（手順 24・`NoSpeedWatch`、スタブ 60 s 完走 終了コード 0・D 無視で中止→N→終了コード 5、テスト通過・ruff OK）。机上まで完了。段6 はハード取り付け待ち |
