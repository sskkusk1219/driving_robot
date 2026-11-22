---
name: bench-verification
description: driving_robot アプリの起動・動作確認の手順。スタブHW／ベンチ（GPIOのみ実機）／実機の3モードの使い分け、環境変数、安全上の禁止事項を定義する。アプリを起動して動作確認したい、WebUI/APIの変更を検証したい、uvicorn を立ち上げたい、実装後の verify をしたい、というときは必ずこのスキルを使うこと。環境変数を誤ると実機アクチュエータが動くため、スキルなしで起動コマンドを組み立てない。
---

# ベンチ/実機検証（bench-verification）

FastAPI アプリ（WebUI）を安全に起動して動作確認するスキル。
対象は実車のペダルを物理アクチュエータで操作するロボットなので、**モード選択を誤ると実機が動く**。

## 今の作り（2026-11-15 段7）

- 制御・学習・スケジュール再生は Web サーバー（uvicorn）の中では動かない。`src/app/job_manager.py` の `JobManager` が
  **ランナー（`python -m src.robot.run`）を別プロセスで 1 本ずつ起動**し、標準出力のイベントで状態・ライブ値・ログを受け取る
  （詳細は `src/robot/README.md`）。ランナーの中身は `tests/research` の写し（`src/robot/core`）。
- ジョブ: 初期化だけ（手順 1。**手順 1 を行うのは初期化ページ（`POST /drive/initialize`）だけ**）／学習（`2,25`・`12〜14,25`）／モード走行（**常に `24,25`**。手順 3 は使わない）／スケジュール再生・シーケンス／手動運転・キャリブレーション（対話ジョブ）。
- 車は `vehicles/<車ID>/vehicle.yaml`（git 管理外）。画面の「車の選択」は `vehicles/.active` に保存される。
- 実機（`DRIVING_ROBOT_USE_REAL_HW=1`）では JobManager がランナーを **`--hw real`** で起動する。それ以外はすべて `--hw stub`。
- 非常停止（画面のボタン・GPIO・AC 断・`POST /drive/emergency`）→ `JobManager.emergency()` → ランナーへ **SIGINT** →
  終了処理（原点復帰 → サーボOFF → 切断）。40 秒で終わらなければ SIGKILL。解除は `reset-emergency`。

## 3つのハードウェアモード

| モード | 環境変数 | HW | 用途 |
|---|---|---|---|
| **スタブ（既定）** | なし | 全てスタブ | UI/API/ロジックの動作確認。Claude が自律的に使ってよい |
| ベンチ | `DRIVING_ROBOT_USE_REAL_HW=1` + `DRIVING_ROBOT_BENCH_GPIO_ONLY=1` | 非常停止スイッチ(GPIO)のみ実機、アクチュエータ/CANはスタブ | 非常停止スイッチの実機検証（ユーザー立ち会い） |
| 実機 | `DRIVING_ROBOT_USE_REAL_HW=1` | 全て実機（Modbus RTU アクチュエータ・CAN・GPIO） | WebUI での本番走行・実機確認。**Claude も起動・操作してよい**（ユーザー許可 2026-11-15）。起動前に下の「走らせる前の確認」を必ず行う |

**`DRIVING_ROBOT_USE_REAL_HW=1` で WebUI を起動・操作することは Claude にも許可されている**（ユーザー許可 2026-11-15「これからも Claude が操作することを許可する」）。
ただし実機モードではアクセル/ブレーキアクチュエータが物理的に動作し、シリアルポートを占有するので、起動前に「走らせる前の確認」を必ず行い、確認が終わったら uvicorn を止めてシリアルを空ける。ベンチ（GPIO のみ）は非常停止スイッチを押す人が要るのでユーザー立ち会い。
研究用の実機走行（`tests.research.main --hw real`）と、ランナーの直接実行（`python -m src.robot.run --hw real`。WebUI を介さない）は
別扱いで、下の「実機走行」に従えば Claude が無人で実行してよい（どちらも同じ確認・同じ止め方）。

- ボタンなし（車両 yaml `buttons.installed: false`）のときは Ready・D・N・IGN-OFF を押したことにするだけ（ログに「ボタンなし:」）。
  非常停止(GPIO)の見張りは `config/settings.toml` の `[web_gpio] enabled = false` か環境変数 `DRIVING_ROBOT_GPIO_DISABLED=1` で切る。

## DB の有無

- `DATABASE_URL` 未設定 → **in-memory リポジトリ**（モード・スケジュール・シーケンス・セッションは揮発。走行ログの取り込みは無し）。
  純粋な UI 動作確認はこれで十分なことが多い。車（プロファイル）は DB ではなくフォルダなのでこの違いと無関係。
- `DATABASE_URL=postgresql://localhost/driving_robot` → 本番 DB に接続。
  起動時に `running` のまま残った孤児セッションを `error` に是正する副作用があるので、
  **本番 DB を汚したくないテストでは DATABASE_URL を付けずに起動する**のが安全。
  走行（モード走行・スケジュール）が DB に残ることまで確かめたいときは、**本番 DB の複製**を作る:
  `createdb -T driving_robot driving_robot_check`（確認後に `dropdb driving_robot_check`）。
- **エラーは `error_events` へ**（エラーページ。RPD_Phase_20261120_ErrorLog）: ジョブの終わり・非常停止・待機中の警告/エラー・計画の準備の失敗・
  シーケンスの停止・サーバーの WARNING 以上・ブラウザの赤いトーストが、サーバーを止めても残る。DB 未設定（in-memory）の起動では揮発。
  `DATABASE_URL` を付けて起動すると、起動時に `error_events` を `CREATE TABLE IF NOT EXISTS` で用意する（本番 DB でもテーブルが増えるだけ）。
  本番 DB を汚したくない検証でエラーページの記録まで見るときは、DB の複製（`driving_robot_check`）を使う。
- 車両フォルダ: 既定は `vehicles/`。走らせると `vehicle.yaml`・`data/` が書き変わりうるので、検証では
  scratchpad に複製して `DRIVING_ROBOT_VEHICLES_DIR=<複製の親フォルダ>` で指す（終わったら消す）。共通設定は `DRIVING_ROBOT_COMMON_CONFIG`（既定 `config/robot_common.yaml`）。

## 起動手順

1. **ポート確認（必須）**: `ss -tln | grep -E ':8080'`
   - 8080 が使用中なら、それは**ユーザーが手動起動したインスタンスの可能性があり、実機接続かもしれない**。
     勝手に走行系 API を叩かない・kill しない。自分の検証には別ポート（8081）で新規に立てる。
2. **スタブモードで起動**（バックグラウンド）:
   ```bash
   DATABASE_URL=postgresql://localhost/driving_robot_check \
   DRIVING_ROBOT_VEHICLES_DIR=<scratchpad>/veh \
     .venv/bin/uvicorn src.web.app:app --host 0.0.0.0 --port 8081
   ```
   （DB なし・車両は既定のままなら環境変数なしでよい）
3. **疎通確認**: `curl -s http://localhost:8081/api/v1/drive/status`
   （`/` は 307 → `/static/index.html`）
4. **変更系の API（GET 以外）は「操作」のウィンドウからしか通らない**（2026-11-22 操作権。`src/web/control.py`）。
   curl の POST は 403「閲覧用のウィンドウでは操作できません」になる。Playwright で画面を開いて「操作」を選び、
   そのページで `fetch(path, {method:'POST', headers:{...window.drvControlHeaders(), 'Content-Type':'application/json'}, body})` で叩く。
   - 操作権は 1 つだけ。**ユーザーが別の画面で「操作」を持っていると Claude は操作できない**（奪い取りはない）→ 止めてユーザーに頼む。
   - 操作のタブを閉じてから約 10 s は操作権が空かない（読み直しを待つ）。2 タブ目は `isolatedContext` を分けると別のウィンドウになる。
   - GET（状態・ログ・CSV）は curl のままでよい。
   走行系の確認（スタブ）: `POST /api/v1/drive/select-profile {"profile_id":"<車ID>"}` → `POST /drive/initialize`（手順 1。**これが済むまで学習・モード走行は 409。`GET /drive/status` の `initialized` が true になる**）→
   `POST /drive/start {"mode_id":"…","limit_s":60}`（モード走行・手順 24,25）→ `GET /drive/status` の `job`/`last_job` を見る
   （`exit_code` 0・`result.meaning` 成功）。スケジュールは `POST /drive/schedule/start {"schedule_id":"…"}`。
   スタブの車は応答が実車と別物（制御ゲインの判断には使えない）。流れの確認だけに使う。
5. **検証後は自分が立てた uvicorn を必ず停止する**（`pkill -INT -f "uvicorn src.web.app:app --host 0.0.0.0 --port 8081"`。
   ポートとプロセスを残さない）。作った DB の複製・車両の複製も消す。

## WebUI の確認方法

- **対話的な確認・スクショ**: Playwright MCP（`mcp__playwright__browser_navigate` → `browser_snapshot` →
  `browser_take_screenshot`）。SSH/ヘッドレス環境なのでブラウザ表示はない。スクショを Read で確認する。
- **スクリプト化した再現テスト**: `webapp-testing` スキル（Python Playwright、`headless=True` 必須）。
- WebUI コンポーネントの罠（共有 Box の onClick、Btn の boolean prop 等）は既知の注意点がある。
  挙動が不可解なときは `src/web/static/js/` の該当コンポーネント実装を確認する。

## テストとの使い分け

- ロジックの検証はまず `pytest tests/unit/`・`pytest tests/integration/`（統合テストはローカル PostgreSQL 使用）。
  `tests/research` は変更しない（移植元）。src 用のテストは `tests/unit` に置く。
- このスキルの出番は「実際にアプリを立ち上げて UI/API を通しで確認する」とき。
  テストが通っただけで「動作確認済み」と報告しない。
- WebUI の実機モードの検証は計画だけ立ててユーザーに依頼する。研究用の実機走行は下の節に従って Claude が行う。
  結果の解析は `drive-log-analysis` スキルで行う。

## 実機走行（tests/research・src ランナー）— Claude が無人で実行してよい（ユーザー決定 2026-10-18）

ユーザーの立ち会い・合図は不要。実装計画の最後の実機確認は Claude が走らせて解析し、結果を報告する。

### Claude が実行してよいもの（人の操作・目視が要らないもの）

| コマンド | 内容 |
|---|---|
| `.venv/bin/python -m tests.research.main --steps 1,2 --hw real; echo "exit=$?"` | 手順2（学習運転 → FF モデル → yaml 書き戻し） |
| `.venv/bin/python -m tests.research.main --steps 1,3 --hw real; echo "exit=$?"` | 手順3（WLTP モード走行）。`--limit-s N` 付きも可 |
| `.venv/bin/python -m tests.research.main --steps 1,2,3 --hw real; echo "exit=$?"` | 手順2 → 手順3 |
| `.venv/bin/python -m tests.research.main --steps 1,11 --hw real; echo "exit=$?"` | 加振（周波数応答の測定） |
| `.venv/bin/python -m tests.research.main --retrain <CSV> --hw real` | 走らずにモデルを作り直して yaml へ書き戻す |
| `.venv/bin/python -m src.robot.run --vehicle <車ID> --steps 1,24 --mode <モード名> --hw real; echo "exit=$?"` | src ランナーでモード走行（手順 24。WebUI が実機でやるのと同じ中身）。`--limit-s N` 付きも可。手順 2 は `--steps 1,2`、スケジュールは `--schedule-file` |
| `sudo .venv/bin/python scripts/check_can.py` など読むだけの確認 | CAN 受信・UPS（チェック1・2）。終わらないものは timeout を付ける |

### ユーザーに頼むもの（人の操作・目視が要る）

- `tests/hardware/test_calibration.py`（ペダル位置の合わせ込み。ジョグ＋目視で Enter）と、その 4 値の yaml への書き写し
- 非常停止スイッチの押下: `tests/hardware/test_emergency_stop.py`（GPIO だけ）。走行中に押して原点復帰まで見るのは、WebUI の実機起動（ユーザー）か、ランナー直接実行の最中にユーザーが押す
  （旧 `test_emergency_stop_home_return.py`・`test_overcurrent_home_return.py` は古い src のアクチュエータ用だったので削除）
- `tests/hardware/test_servo_pca9685.py`（サーボの動きを目視）、UPS チェック3（AC ケーブルを抜く）
- ベンチ起動（GPIO の非常停止スイッチを押す確認。WebUI の実機起動そのものは Claude が行ってよい）

### 走らせる前の確認（毎回）

1. `ss -tln | grep -E ':8080'` — ユーザーの WebUI が動いていたら走らせない（シリアルを取り合う）。ユーザーに伝えて止める
2. `fuser /dev/ttyUSB0 /dev/ttyUSB1` — 他のプロセスがアクチュエータのシリアルを使っていたら走らせない
3. `pgrep -af "tests.research.main|src.robot.run"` — 前の走行（ランナー）が残っていたら走らせない
4. 変更箇所のテスト・ruff・スタブ（`--hw stub`）が通っていること

### 走らせ方

- CLAUDE.md の指示どおり、実機走行はサブエージェント（haiku）で行い、結果を opus が受け取る。
- 手順2・手順3 は 10 分を超えるので Bash の `run_in_background` で起動し、出力をファイルに残す
  （例: `... --hw real > <scratchpad>/run_<日時>.log 2>&1; echo "exit=$?" >> <同じファイル>`）。
- 終了コード（0 成功 / 2 設定 / 4 初期化 / 5 走行エラー / 6 モデル作成 / 7 走行前チェック / 8 手順2 の学習結果 NG）を必ず報告する。
- yaml の `candidate`・`model_path`・`vehicle` を Claude が手で書き換えない（プログラム自身の書き戻しは可）。
  比べたい制御は候補名で切り替える。

### 途中で止めるとき

- **`kill -INT <pid>`（Ctrl+C と同じ）だけを使う**。`main` の `finally` が原点復帰・切断・ログ保存をする作り
  （ランナーはスタブで SIGINT から原点復帰までの動きを確認済み。実機は未確認）。WebUI の停止・非常停止も同じ SIGINT で、
  終わらなければ 40 秒後に SIGKILL する（実機でこの秒数を測る場合は段8）。`kill -9`・`SIGTERM` は `finally` が走らずペダルを踏んだまま残るおそれがあるので使わない。
- 止める基準: 想定時間の 1.5 倍を超えても終わらない、ログが 60 秒以上進まない。
  中断したら、ユーザーへの報告に「中断した・理由・その時の出力の最後」を書く。

### 走った後

- `tests/research/results/drive_log_real_<日時>.csv`（src ランナーは `vehicles/<車ID>/data/` の CSV。モード走行の CSV は WebUI 経由なら DB に取り込まれて消える）を解析する
  （DB を見るなら `drive-log-analysis` スキル。DB の `drive_logs.detail` に約 100 列、ログ画面の CSV ダウンロードで全列）。
- 結果は ProblemReport の該当する段の「結果」に、CSV 名を出典にして書く（`problem-report` スキル）。
- 「直った」は生データで確かめてから書く。
