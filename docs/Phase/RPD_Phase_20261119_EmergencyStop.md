# 非常停止の挙動の変更（RPD_Phase_20261119_EmergencyStop）

作成日: 2026-11-19 / 対象: `driving_robot`

## 1. なぜ変えるか

今の非常停止は、どのきっかけでも「その場で両ペダルを原点復帰 → サーボOFF → 切断」で終わる。
- 走行中だと、ペダルを離した車が惰行で減速する。
- 手順 24 では、離した後にいきなり停車保持の開度まで踏んで N を押す。
- 車は Ready・D のまま残る。

これを「減速して停車させる → N → 停車を確かめてブレーキを離す → スタートを 10 秒押して IGN-OFF → 原点復帰」に変える。あわせて、緩減速（`decelerate_to_stop`）の踏み込みの上限が停車保持開度になっていた欠陥も直す。上限は max 開度にする。

## 2. 決定事項（2026-11-19 の議論）

| # | 決定 |
|---|---|
| 対象 | 走行するジョブ（手順 2・12〜14・24、スケジュール）。対話のジョブ（手動・キャリブレーション）と、ボタンだけのジョブ（20〜23・25）は今どおり |
| きっかけ | SIGINT（非常停止スイッチ・画面の非常停止・停止ボタン・AC 断）と、走行中の自動の非常停止（DriveError） |
| 通常 | 車速 > 0.2 km/h なら 0.2G で停車させる → 停車保持開度 → N → 0.2 km/h 以下か確かめる。以下でなければさらに踏んで N（最大 3 回）。0.2 km/h 以下になったらブレーキを離す → スタート 10 秒 → 原点復帰 |
| 停車中 | 停車中も先に N を押す（停車保持開度 → N → 確かめる → 離す → スタート 10 秒 → 原点復帰）。D に入っていたときのクリープを防ぐ |
| さらに踏む量 | 停車保持開度から max 開度までを 3 等分し、1 回ごとに 1 段深くする。3 回目でちょうど max |
| 3 回でも止まらない | ブレーキを max で踏んだまま終える。スタートは押さない。原点復帰・サーボOFF はせず、**サーボ ON のまま**接続だけ閉じる。警告を出す |
| CAN を読めない | アクセルを原点へ → ブレーキを `emergency.can_lost_press_s`（既定 20 秒。min から max までの秒数）かけて max まで刻んで踏む → 10 秒保つ → N → 離す → スタート 10 秒 → 原点復帰 |
| アクチュエータの異常 | ブレーキ軸の異常なら、両軸をすぐ原点復帰 → スタート 10 秒。アクセル軸だけの異常なら、アクセルを原点へ戻し、ブレーキで「通常」（CAN を読めなければ「CAN を読めない」）の流れ |
| 2 回目の押下 | Web が EMERGENCY の間は 2 回目を無視する（今どおり）。自動の非常停止の流れの最中に停止・非常停止が押されても、**動いている流れを続ける** |
| 緩減速の上限（D） | どこで使うときも max 開度。停車した後は停車保持開度へ移す（戻す向きも動かす） |
| 猶予 | SIGKILL までを 40 秒から 120 秒へ |
| 画面 | 赤い EMERGENCY 画面などの**文言だけ**直す（レイアウト・色・ボタンは変えない） |
| ドキュメント | product-requirements の「即座に原点復帰」を書き換える |

実装で足した決め（計画にない細部）:
- 走行前チェックが済む前（手順 1 の途中など）の SIGINT では流れを走らせない。車が IGN-OFF のままのことがあり、スタートを押すと IGN-ON になるため。従来どおり原点復帰・サーボOFF だけ行う。
- N の確認は「停車保持開度〜max 開度を 3 等分した 4 つの開度」で最大 4 回押す（決定事項の「3 回」は、停車保持での最初の 1 回に続く踏み増しが 3 回の意味）。どの回も今のブレーキ位置より浅くはしない。

## 3. 作り

core（`src/robot/core` ＝ tests/research の写し）は、緩減速の上限の修正（D）だけ直して写し直す。
非常停止の流れは、src だけのコードとして `src/robot/emergency_stop.py` に置き、差し替えは `patches.py`（差し替え 10）に集める。

### 段 1: 緩減速の上限（tests/research → 写し直し）
- `tests/research/stop_decel.py`: 踏み増しの上限を `profile.max_brake_opening` にする。停車後は今の位置から停車保持開度へ刻んで移す（`step_to_position` は戻す向きにも動く）。docstring・表示文を直す。
- `tests/research/test_research_stop_decel.py`: 上限が max 開度・停車後は停車保持開度で終わること。
- `scripts/port_research.py` で写し直し、`--check` で一致を確かめる。

### 段 2: 非常停止の流れ本体 `src/robot/emergency_stop.py`（新規）
- 状態はプロセスで 1 個: `enabled`・`ctx`・`task`・`brake_kept`・`handled`。
- `run(hw, reason)`: 流れを `asyncio.shield` した 1 本の task で走らせる。2 回目は同じ task を待つだけ。
- `classify`: CAN 車速 → 各軸の `read_monitor`・アラーム → 中断理由の文言（「アクセル軸」「ブレーキ軸」＋過電流・アラーム・電流）の順で、流れを決める材料を集める。
- 再利用: `decelerate_to_stop`・`step_to_position`・`press_button`・`set_brake`・各軸の `home_return`。
- 段ごとに `events.emit("state", phase="emergency", stage=..., detail=...)`。スタート 10 秒を押し終えたら `stage="ign_off"`。止まらないときは `stage="brake_kept"`。

### 段 3: 差し替えとランナー
- `patches.py` 差し替え 10: `_release_pedals`（core の pattern_drive・mode_drive・excite と src の schedule_job）・`press_shift_n_after_drive`・`shutdown` を包む。
- `run.py`: `emergency_stop.configure(ctx, enabled=...)`（手順 2・12〜14・24 かスケジュールを含む非対話ジョブ）。
- `config_store.py`: src 専用キー `emergency.can_lost_press_s`（既定 20.0）。
- `job_manager.py`: `KILL_TIMEOUT_S = 120.0`、`phase=emergency` の `ign_off` で `ign_off_done`、`brake_kept` で警告、状態に `emergency_stage`・`brake_kept` を足す。

### 段 4: 画面の文言・ドキュメント
- `system-states.js` の `EmergencyResetScreen`・`job-utils.js` の `PHASE_LABEL`・非常停止の説明文（文言だけ）。
- `docs/product-requirements.md`・`docs/hardware.md`・`src/robot/README.md`。

## 4. 検証

**机上・スタブ（Claude が自動で）**
- `tests/unit/test_emergency_stop.py`（偽の HW）: 走行中 / 停車中 / 止まらない / CAN を読めない / ブレーキ軸の異常 / アクセル軸だけの異常 / 2 回目の呼び出し・cancel / 文言の見分け / 差し替え。
- スタブの通し: `--steps 1,24 --hw stub` で走行中・停車中に SIGINT。

**実機（Claude が進め、人手が要る所だけユーザーに指示）**

| # | ユーザーにしてもらうこと | 確かめること |
|---|---|---|
| 1 | 手順 24 の走行中に、指示した時刻で**非常停止スイッチを押す** | 0.2G 前後で停車 → N → IGN-OFF → 原点復帰、赤い画面の文言・段の表示、120 秒以内 |
| 2 | 停車中（走行前チェックの後・D の直後）に**非常停止スイッチを押す** | N → 離しても動かない → IGN-OFF |
| 3 | 走行中に **CAN ケーブルを抜く** | max まで刻んで踏む → 10 秒 → N → IGN-OFF（終わったら挿し直してもらう） |
| 4 | 走行中に **UPS の AC を抜く** | #1 と同じ流れ（AC 断 → 非常停止） |
| 5 | 毎回、車の状態（State・Range）を**目視**で教えてもらう | IGN-OFF・P になっているか |

- 学習の手順（2）の終わりの緩減速で、上限が max になったこと（停車保持開度を超えて踏める）を走行ログで確かめる。
- アクチュエータの異常は実機で起こせないため、単体テストだけで確かめる。
