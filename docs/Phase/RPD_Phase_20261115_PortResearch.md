# tests/research の制御を src（本番）へ移植・GUI 化 実装計画

作成日: 2026-11-15 / 対象: `driving_robot`
目的: `tests/research` の制御（正しい制御）を `src/` に移し、WebUI でロボットを設定・操作できるようにして、研究を知らない人でも使えるようにする。

---

## 0. 前提（ユーザー指示）

- `tests/` のコードは変更しない（移植元として読むだけ）
- `src/` のアクチュエータ制御のコードは捨ててよい。`tests/research` の制御が正しい
- `src/` のフロントエンドはできるだけ今のまま使う。合わないところは作り替える
- `tests/research` は yaml 1 本で管理しているが、`src/` は**車ごとに yaml** を作る。共通の項目は別のファイル（拡張子は問わない）で持ち、ユーザーには見せない

## 1. 今の違い（調査）

| | `src/`（本番） | `tests/research` |
|---|---|---|
| 規模 | 約 1.7 万行 | 約 10 万行（テスト込み）・yaml 1438 行 |
| 開度の定義 | キャリブレーション（ゼロ点・フル点） | 0% = 原点復帰位置・100% = 9500 pulse、min（ペダルに触れた位置）は手で測る |
| 学習 | 学習運転 ＋ PID 自動適合（学習サイクル） | 手順2: ペダル探索 → G ランプ → 表・NN の pkl → 学習結果チェック |
| 車の設定 | DB の profiles（JSONB） | yaml ＋ pkl |
| 走行ログ | DB（drive_logs） | CSV・PNG・レポート md |
| 実行 | uvicorn 内の asyncio ループ | CLI の手順 0〜25 |

## 2. 決定事項（2026-11-15）

ユーザー決定は A-2 のみ。他は「Claude のおすすめでよい」（ユーザー、2026-11-15）。

### A. 範囲

| # | 項目 | 決定 |
|---|---|---|
| A-1 | 持ってくる手順 | 手順1（初期化）・走行前チェック・手順2（学習）・手順3（モード走行）・手順20〜25（ボタン操作）。手順12・13・14（今の pkl への部分的な学習のやり直し）も学習画面から実行できるようにする。手順4〜11（PID 適合・加振・制約）と分析・デバッグの道具は持ってこない |
| A-2 | 手順3 のレポート（md・図） | **作らない。DB のログとログ画面だけで十分**（ユーザー決定） |
| A-2b | ログの残し方 | **DB だけ**（ユーザー決定 2026-11-15）。走行ログは tests の CSV と同じ約 100 列を全部 DB（今の 9 列＋残りは `detail JSONB`）。進行のメッセージも走行ごとに DB（`drive_session_messages`）。CSV ファイルは残さない（ログ画面の CSV ダウンロードで DB から全列を書き出す） |
| A-3 | 「tests/ は変更しない」の範囲 | **tests/research だけ**（ユーザー決定 2026-11-15）。tests/unit の旧テスト（消す古い src を試すもの）は段7 で古いコードと一緒に消す。新しい src のテストは tests/unit に置く |

### B. 移し方

| # | 項目 | 決定 |
|---|---|---|
| B-3 | 数百ある on/off | まず**今の組み合わせのまま忠実に移す**。同じ入力から同じペダル指令が出ることをスタブで突き合わせて確かめる（等価確認）。採用済みだけに整理するのは等価確認の後の別の段 |
| B-4 | 今後 tests で改良したとき | src のモジュール構成を tests とほぼ 1 対 1 に保ち、差分で取り込めるようにする。各ファイル先頭に「移植元」を書く（tests → src の逆向きの README と同じ形） |
| B-5 | 捨てる src のコード | 制御（`drive_loop`・`learning_loop`・`pedal_plan`・`plan_update`・`trim`・`reward`・`pid`・`feedforward`・`plan_service`・`pid_tuning`・`learning_cycle`・`model_training`・`learning_drive` など）は捨てる。残すのはキャリブレーション（min 測定に作り替え。D-13）・手動運転・タイムスケジュールとシーケンスの実行（tests の制御に乗せ替え）・モード・セッション・ログ・UPS |
| B-6 | 制御ループの動かし方 | 50ms の制御（NN の推論を含む）は Web サーバーとは別のプロセスで動かす。Web 側は指令（開始・停止・非常停止）と状態・ログだけをやり取りする |

### C. 設定ファイル

| # | 項目 | 決定 |
|---|---|---|
| C-7 | 分け方 | ①**車両 yaml**（車ごと）: 諸元・min/max 開度・手順2 が書く値・model_path・ボタン・走行抵抗 ／ ②**共通設定**（ユーザーに見せない）: 制御の on/off・車に依らない固定値 ／ ③`config/settings.toml`（今のまま）: ポート・CAN・GPIO。約 400 キーの振り分けは Claude が案を作り、計画に表で載せる |
| C-8 | 試験車で合わせた値（`deviation_c`・`ff_hf_div` など） | 今は共通の固定値として扱う。計画に「試験車で合わせた値」の一覧を残し、別の車で走るときの見直し対象にする |
| C-9 | ボタン（サーボの角度・押す秒数・回数） | 車で変わるので車両 yaml |
| C-10 | DB の profiles テーブル | 使わない（車両 yaml に置き換え）。既存の DB のプロファイルは捨てる。セッションの profile_id は車両 yaml の名前にする |
| C-11 | 置き場所 | 車両 yaml・pkl は git 管理外のフォルダ（1 台 1 フォルダ）。今の試験車の yaml・pkl を最初の 1 台として移す（変換は src 側の道具で行い、tests の yaml は書き換えない） |

### D. 画面（今の画面を作り替える所）

| # | 画面 | 決定 |
|---|---|---|
| D-12 | プロファイル | 車両 yaml の「人が決める値」（名前・諸元・min/max・ボタン）を読み書きする。学習で決まる値は読むだけで表示する。PID・FF パラメータの欄は消す |
| D-13 | キャリブレーション | ゼロ点・フル点の設定をやめ、ジョグで min（ペダルに触れた位置）を測って車両 yaml に保存する画面にする |
| D-14 | 学習運転 | 段階表示を手順2 の段階（探索 → G ランプ → モデル作成 → チェック）にする。チェックが NG のときは yaml を書き換えず（tests と同じ）、どのチェックが NG かと「やり直し」を出す。手順12〜14 の部分学習もここから |
| D-14b | 学習運転のボタン操作 | **src の学習運転（手順2・12〜14）も、手順24 の開始と同じく Ready・D を押してから学習の走行を始め、終わりに N**（ユーザー決定 2026-11-15）。tests の手順2 にはボタン操作が無いので、src 側（ランナーの差し替え）で足す |
| D-15b | 走行・学習のボタンの流れ | **毎回、通しの流れ**（ユーザー決定 2026-11-15）: モード走行は `1,21,22,24,25`（IGN-ON・メンテナンスモード → N ホールド → Ready・D・モード・N → IGN-OFF）、学習運転は `1,21,22,` Ready・D → 学習の手順 → N `,25`。シーケンスのモードの段も同じ。毎回 IGN-OFF で終わるので続けて走らせても車の状態がずれない（`1,24` を続けると 2 本目の Ready 押しで IGN-OFF になり D に入らない。段8 の US06 101710 で起きた） → 2026-11-15 夕方ユーザー決定: 21（メンテナンス）・22（N ホールド）は通しの流れから外す。モード `1,24,25`・学習 `1,<学習の手順>,25`。手順 25 はブレーキ ON でスタート 10 s（`buttons.ign_off_press_s`、差し替え 7。README の表で 6 は学習のボタン操作のため 7 にした） |
| D-15c | 初期化・ボタンなし（2026-11-16 ユーザー決定） | **手順1（初期化）は初期化ページだけで行う**。モード走行・学習のジョブは初期化の表示・確認をせず、動かすのに要る接続・サーボON だけを裏で行い（原点は走行前チェックが確かめる）、初期化ページで通っていなければ開始できない（やり直しはサーバー起動後・非常停止後・エラー終了の後）（案ア）。手順は モード `24,25`・学習 `<2\|12\|13\|14>,25`（Ready・D・N はランナーが足す）。メンテナンスモード・N ホールド（21・22）はタイムスケジュール・シーケンスで行うもので、走行モード・学習には入れない。**ボタンなしの設定**（車両 yaml の src 専用キー）: Ready・D・N・IGN-OFF を押したことにしてログに残すだけ（SG90 に接続しない）。非常停止（GPIO）の見張りも設定で切れるようにする。UPS は既に off。走り終わった後のクリープはそのままでよい |
| D-15 | 自動運転 | プランの有効・無効・リセットは消す。開始ボタンに手順20〜25（Ready・D・終わりに N）を組み込む。~~ボタン操作の有無は車両 yaml で切り替え~~ → **src のモード走行は常に手順24（Ready・D・終わりに N）。手順3 は src では使わない**（ユーザー決定 2026-11-15）。シーケンス・スケジュールの実行もこの制御で |

### E. だれでも使えるために

| # | 項目 | 決定 |
|---|---|---|
| E-16 | 使い手 | 研究を知らない担当者が LAN 内の PC・タブレットのブラウザで使う（認証なし）。新しい車は案内型: 車を登録 → min を測る → 手順2 → 結果確認 → 走行。エラーは「何が起きたか・次に何をするか」を日本語で |
| E-17 | 安全 | tests の判定（`axis_safety`・走行前チェック）に、src の非常停止（GPIO・画面のボタン）を足す |

### F. その他

| # | 項目 | 決定 |
|---|---|---|
| F-18 | 依存関係 | `requirements.txt` が無いので作る（torch などを含む） |

## 3. 計画（2026-11-15 承認）

### 3.1 調べて分かったこと（設計の前提）

- `main.py` から辿れる研究モジュールは 76 本・約 5.5 万行。全部 asyncio、出力は `term.say()`（print）。
- `run_steps()`（`tests/research/main.py:1414`）は**毎回** 初期化 → 走行前チェック → 手順 → 必ず `shutdown()`（原点復帰・サーボOFF・切断）。HW を走行の間ずっと握る作りではない。
- yaml への書き戻しは `ResearchConfig.save()` だけ（`config.py:1696`）。書くキーは `feedforward.*`（`FF_PARAM_KEYS`・`model_path`・`model_*_features/horizons`・`creep_speed_kmh`・`stop_brake_offset_pct`）と手順0 の `pid.kp`。
- 走行ログは `SessionLog`（`drive_log.py:315`）が `record()` で集め、`close()` で CSV を書く。`LivePlot` は `output.plot` で無効にできる。
- スタブ車両（`hardware.py`・`vehicle_sim.py`）に乱数は無い（決定的。ずれるのは時刻のゆらぎだけ）。
- 本番 DB の `drive_sessions.profile_id` は `vehicle_profiles(id)` への外部キー（C-10 で変える）。
- 本番のタイムスケジュールは「ペダル開度＋ボタン（PCA9685 の ch 指定）の開ループ再生」。tests に同じものは無い。tests のボタンは `servo_map`（名前 → ch＋角度）。
- torch・numpy・scipy・sklearn・matplotlib は .venv に入っている。`requirements.txt` は無い。

### 全体の形

```
ブラウザ ──HTTP/WS── Web サーバー（uvicorn, src/web）
                        │  ジョブを 1 本ずつ起動（別プロセス）
                        ▼
                  ランナー（python -m src.robot.run …）
                   = tests の main.run_steps と同じ流れ
                     初期化 → 走行前チェック → 手順 → 必ず shutdown
                   標準出力: say() の行 → 画面の「現在ステータス」
                   イベント用の行（JSON）: 状態・ライブの車速/開度・結果
```

- **ジョブ** = 手順の並び（例: 学習 `1,2`、モード走行 `1,24` または `1,3`、部分学習 `1,13`、手動/キャリブレーションはジョグを受け付ける対話ジョブ）。同時に 1 本だけ。
- **非常停止**: 画面のボタン・GPIO（`src/infra/gpio_monitor.py` を Web 側で使う）→ ランナーへ SIGINT（tests の `finally: shutdown()` で原点復帰）→ 決めた秒数で終わらなければ SIGKILL。
- **ログは DB だけ**（ユーザー決定 2026-11-15）。ランナーの `SessionLog` を差し替えた `StreamingSessionLog`（`record()` でライブ値を 5 Hz で流す）。
  - 走行ログ: tests の CSV と同じ約 100 列を全部 DB へ。今の `drive_logs` の 9 列はそのまま、残りの列は新しい `detail JSONB` 列に 1 行ずつまとめる。CSV ファイルは残さない（必要なら既存のログ画面の CSV ダウンロード `/api/v1/sessions/{id}/logs.csv` が DB から全列を書き出す）。
  - 進行のメッセージ（`say()` の行）: 新しい表 `drive_session_messages`（セッション ID・連番・経過秒・時刻・レベル・本文）に残し、ログ画面でセッションごとに読める。
  - ジョブの終わり（異常終了でも）に、ランナーが書いた一時 CSV を Web 側が読んで DB へ入れ、一時ファイルは消す。レポート md・図は作らない（A-2）。
  - 目安: 30 分の走行 1 本で DB が数十 MB 増える。

### ディレクトリ・設定（C-7〜C-11）

| 場所 | 中身 | git |
|---|---|---|
| `src/robot/core/` | tests/research の**写し**（手で直さない。移植道具が作る） | 管理 |
| `src/robot/`（core 以外） | ランナー・ジョブ・設定の合成・差し替え（src だけのコード） | 管理 |
| `config/robot_common.yaml` | 共通設定（制御の on/off・車に依らない固定値）。ユーザーに見せない | 管理 |
| `config/settings.toml` | ポート・CAN・GPIO（今のまま） | 今のまま |
| `vehicles/<車ID>/vehicle.yaml` | 車ごとの値 | **管理外** |
| `vehicles/<車ID>/models/` | pkl（学習結果） | **管理外** |

- **車両 yaml に置くキー**: `vehicle.*`（走行抵抗を含む。`road_load_file` は不要になる）、`buttons.*`（`servo_map` を含む。ボタンの有無 `buttons.enabled` を新設＝src 側でのみ解釈）、手順2・12〜14 が書く `feedforward.*`（上の一覧）。それ以外はすべて共通。振り分けは段1 で全キーの表を RPD に載せる。
- **合成**: 共通 → 車両の順に辞書を重ね、`core.config` の検証をそのまま通す。`results_dir`・`model_path` の置き場所は車のフォルダへ、`output.plot` は false に固定。
- **保存**: core が書く先は合成 yaml。`save()` の後に、書いたキーを車両 yaml へ写す（`core.config.update_yaml_values` を使う）。書いたキーが車両 yaml に無ければエラー（共通設定を勝手に変えない）。
- **C-8**: 試験車で合わせた値（`deviation_c`・`ff_hf_div` など）の一覧を段1 で RPD に残す。

### 段

各段: ruff check・変更箇所のテスト・スタブ確認を Claude が行い、OK なら次へ（止めない）。実装は sonnet のサブエージェント、実機走行は haiku のサブエージェント（結果は opus で受け取る）。

| 段 | 内容 | 主なファイル |
|---|---|---|
| 0 | `requirements.txt` を作る。移植道具 `scripts/port_research.py`: 決めた一覧（手順 1・走行前チェック・2・3・12〜14・20〜25 から辿れるもの。`main.py` を含む。`test_*`・分析・デバッグ用は除く）を `src/robot/core/` へ写し、`tests.research` → `src.robot.core` に置き換え、先頭に「移植元・写した日・tests の commit」を書く。`"tests/research"` の文字列は一覧に出して確かめる。`tests/unit/test_no_tests_import.py`: src に `tests` の import が無いこと | `scripts/port_research.py`, `src/robot/core/*` |
| 1 | 設定の分割と合成・保存（上記）。移行道具 `scripts/import_research_vehicle.py`: 今の `config_testVehicle.yaml`・`private/road_load.yaml`・参照している pkl から `vehicles/test_vehicle/` を作る（tests 側は読むだけ）。**等価確認①**: 合成した設定の `ResearchConfig` が、tests の `load_config()` と（置き場所のキー以外）一致 | `src/robot/config_store.py`, `config/robot_common.yaml` |
| 2 | ランナー: `src/robot/run.py`（引数で車・手順・モード ID・HW の種類）、`StreamingSessionLog`（core の `main` が使う `SessionLog` をここで差し替える。差し替え点は 1 か所だけにして README に書く）、イベント行の形式、対話ジョブ（ジョグ・原点・min の保存）。**等価確認②**: スタブで tests の `main --steps 1,3 --limit-s 60` と src のランナーを走らせ、ペダル指令の列を突き合わせる（時刻のゆらぎ以外で差が無いこと） | `src/robot/run.py`, `src/robot/jobs.py`, `src/robot/streaming_log.py` |
| 3 | Web 側: `JobManager`（起動・状態・ログの行バッファ・ライブ値・非常停止・終わりの DB 書き込み）。`RobotController`・学習サイクル・プラン・PID 適合とその API を外す。DB: `drive_sessions` に `vehicle_id TEXT` を足し `profile_id` を NULL 可に（既存のプロファイルは使わない）、`drive_logs` に `detail JSONB`、表 `drive_session_messages` を新設、CSV ダウンロードは detail を展開して全列、ログ画面にメッセージの表示を足す、run_type は今の値を使う（学習＝learning、モード走行＝auto、手動＝manual） | `src/app/job_manager.py`, `src/web/routers/drive.py`, `scripts/setup_db.py` |
| 4 | 車両 API: `/api/v1/profiles/*` を車両 yaml の一覧・作成（共通の雛形から）・読み書き・削除に置き換える（画面の呼び先は変えない。ID は車のフォルダ名）。人が決める値だけ書ける・学習の値は読むだけ | `src/infra/vehicle_store.py`, `src/web/routers/profiles.py` |
| 5 | 画面（D-12〜15・E-16）: プロファイル（PID・FF 欄を消し、諸元・min/max・ボタンを表示）、キャリブレーション（ゼロ/フル → ジョグで min を測って保存）、学習運転（手順2 の段階表示・チェック NG の表示とやり直し・部分学習 12〜14）、自動運転（プラン・一時停止を消す。ボタン操作ありは手順24、なしは手順3）、初期化（ジョブの中の初期化の進み具合を表示）、ホームに新しい車の案内（登録 → min → 学習 → 確認 → 走行）。エラーは「何が起きたか・次に何をするか」 | `src/web/static/js/screens/*.js` |
| 6 | タイムスケジュール・シーケンスの実行: スケジュールは core のアクチュエータ・`button_servo` で開ループ再生するジョブを src で書く（本番 `schedule_loop.py` の考え方を移す）。ボタンは ch 指定をやめて車両 yaml のボタン名（スタート・P・N・D）で指す（今の ch のデータは名前へ変換）。シーケンスは段を順にジョブとして走らせる | `src/robot/schedule_job.py`, `schedule-sequence.js` |
| 7 | 古いコードの削除（B-5 の一覧）と、`docs/`（product-requirements・memo_codeRun・hardware）・`bench-verification` スキルの起動手順の更新 | `src/domain/control/*` ほか |
| 8 | 実機確認（下記） | — |

取り込み直し（B-4）: tests を直したら `scripts/port_research.py` を再実行 → 差分を確かめ → 等価確認①② を回す。core は手で直さない。

### 確かめ方

- 各段: `ruff check`、変更箇所のテスト（`tests/unit` に src 用を足す。tests/research には足さない）。
- 等価確認①（段1）・②（段2）は上のとおり。
- 画面: スタブ構成で起動（`bench-verification` スキル）、Playwright で 車の登録 → min の保存 → 学習（スタブ）→ モード走行 → ログ画面に出る、までを通す。非常停止で原点に戻ること。
- **実機（段8・Claude が無人で）**: ① GUI からモード走行 WLTP・UDDS・US06（ボタン操作あり）。比べる相手は同じ日の tests の本体の走行（今日すでに同じ md5 の本体があればそれ）。DB から書き出した CSV（全列）で KPI・往復回数を出し、日のばらつきの範囲か見る（tests の CSV と同じ列なので research-run-analysis の道具で比べられることも確かめる）。② GUI から手順2 を 1 回（`vehicles/test_vehicle` の写しに書かれ、tests の yaml は変わらないこと・学習結果チェック OK）。③ 走行中に画面の非常停止 → 何秒で原点に戻るかを記録。
- ユーザーに頼むのは、キャリブレーション画面で測った min がペダルに触れた位置と合っているかの目視だけ。

## 4. 結果

### 段0（2026-11-15）: requirements.txt・移植道具

| 項目 | 結果 |
|---|---|
| `requirements.txt` | 新規。src・tests/research が実際に import しているライブラリ（fastapi・uvicorn[standard]・pydantic・starlette・asyncpg・pymodbus・pyserial・python-can・cantools・smbus2・numpy・scipy・scikit-learn・torch・matplotlib・PyYAML）と開発用（pytest・pytest-asyncio・httpx・ruff・mypy）を、今の .venv の版で `==` 固定。lgpio は apt、pandas は分析専用なので載せない（コメントで明記） |
| `scripts/port_research.py` | `main.py` から ast で辿れる 76 モジュール（関数内の遅延 import 含む）・合計 54,690 行（移植元の行数）を `src/robot/core/` へ写した。`test_*`・results・private は辿れなかった（写していない）。`tests.research` → `src.robot.core` を import・文字列とも置換。各ファイル先頭に移植元コメント（長い行なので E501 だけ `# noqa`）。再実行で上書き・写し元から消えたファイルは削除。`--check` で写さずに差だけ確かめる。`PORTED.md` に一覧（ファイル・移植元・行数・commit） |
| 確認 | `ruff check src/robot/core` 合格・`import src.robot.core.main` 成功・`port_research.py --check` 一致 |
| `tests/unit/test_no_tests_import.py` | src 配下の全 .py に `import tests`・`from tests` が無いこと（合格） |

#### `"tests/research"` を含む文字列（docstring 以外 10 件）と扱い

| 場所（src/robot/core） | 中身 | 扱い |
|---|---|---|
| `config.py:30` | `DEFAULT_CONFIG_PATH`（既定の yaml） | 触らない。ランナーは `load_vehicle_config` の合成 yaml を必ず渡す。存在しない path を渡したときだけ `ensure_config` が参照するので、段2 のランナーは存在確認してから渡す |
| `config.py:88`・`:749` | `model_path`・`results_dir` の既定値 | `compose()` が必ず上書きする（車両 yaml に model_path が無い車は `models/untrained.pkl`）。等価確認①で上書きを確認 |
| `fwd_map.py:58`・`fwd_tool.py:86`・`debug_process23.py:37,38`・`mode_report.py:1259`・`main.py:143` | CLI の引数の既定値（`MODELS_DIR`・`RESULTS`・`--config` の既定） | ランナー経由では使われない（`save_table` の呼び出し元は main・pattern_drive とも明示パス。`default_table_path`・`model_default_path` は fwd_tool・fwd_final・fel の CLI だけ）。ランナーは `main.py` の `argparse` を通らず `run_steps` を呼ぶ（段2） |
| `mode_report.py:1019` | レポート md に書く説明文の中の `tests/research/kpi.py` | 表示だけ。A-2（レポートは作らない）で使われない |

文字列ではないが同じ種類の注意（段2 以降で扱う）:

- `config.py:1789`（`_apply_road_load_file`）・`mode_report.py:194`（`_REPO_ROOT`）は `Path(__file__).resolve().parents[2]` でリポジトリ直下を求める。core では `src/` になるが、前者は `road_load_file` を無効にするので呼ばれず、後者は git の commit を調べるだけ（レポート不使用）。
- 手順 13・14 の `_pkl_source_csvs` は pkl の meta の `sources`（学習に使った CSV）を `cfg.results_path`（= `.work`）から探す。車両フォルダにはまだ学習の CSV を持っていない。部分学習（手順 13・14）を GUI から動かす段までに、学習 CSV の置き場を決める必要がある（残った問題）。

### 段1（2026-11-15）: 設定の分割・合成・保存

作ったもの:

| ファイル | 内容 |
|---|---|
| `src/robot/config_store.py` | `load_vehicle_config(vehicle_id, *, vehicles_dir, common_path)` → `(ResearchConfig, merged_path)`。共通 → 車両の順に深く重ね、`output.results_dir` = `.work`・`output.plot` = false・model_path 等は `vehicles/<ID>/models/` 基準・`road_load_file` = null にして `.work/merged.yaml` を書き、core の `load_config` + `validate_config` で読む。車両キーは定数 `VEHICLE_SECTIONS`・`VEHICLE_FEEDFORWARD_KEYS` の 1 か所。保存は `sync_saved_to_vehicle`（ランナーが `cfg.save()` の直後に呼ぶ。`install_save_sync` で `cfg.save` を包めば呼び忘れない。`ResearchConfig` は frozen でないので core を直さずにできる）。車両 yaml に無いキー・共通のキーが変わっている場合は例外。`update_yaml_values` で書くのでコメントは保たれる |
| `config/robot_common.yaml` | 共通設定（git 管理。約 1,370 行）。元の yaml から車両キーを除いたもの（コメントは保つ） |
| `vehicles/test_vehicle/vehicle.yaml`・`models/` | 試験車の車両 yaml（約 80 行・git 管理外）と pkl のコピー（`model_path` の 1 個。`_hold`・`_stop` の別名 pkl は無かった）。走行抵抗は private/road_load.yaml の値を `vehicle.*` に直接書いた |
| `scripts/import_research_vehicle.py` | 上の 2 つを作る道具（tests 側は読むだけ。既にあれば `--force` が要る。分割した結果を重ね直すと元の yaml と一致することを毎回検算） |
| `scripts/check_config_equivalence.py` | 等価確認①（下）。tests と src の両方を import するので tests/unit ではなく scripts に置いた（CLAUDE.md「本番とテスト環境の隔離」） |
| `tests/unit/test_robot_config_store.py` | 合成・上書き・保存の写し・例外・車両キーの網羅（`FF_PARAM_KEYS` − `MEASURED_KEYS` と `pedal_search`・`build_ff_model` の保存キーがすべて車両キーに入っている）・共通 yaml に車両キーが無いこと。11 件のうち config_store 分 10 件が合格（残り 1 件は no_tests_import） |
| `.gitignore` | `vehicles/` を追加 |

#### 車両キーと共通キーの振り分け（キー全部。合成した設定の yaml のキーで数えた。車両 42 個・共通 393 個）

| セクション | 車両 yaml のキー | 共通のキー数 | 共通のキー |
|---|---|---:|---|
| vehicle | **全部**（15 個）: `name`, `max_speed_kmh`, `max_decel_g`, `min_accel_opening_pct`, `max_accel_opening_pct`, `min_brake_opening_pct`, `max_brake_opening_pct`, `stop_deviation_threshold_kmh`, `stop_deviation_duration_s`, `test_mass_kg`, `road_load_a0_n`, `road_load_a1_n_per_kmh`, `road_load_a2_n_per_kmh2`, `road_load_source`, `road_load_file` | 0 | なし |
| feedforward | `model_path`, `creep_speed_kmh`, `creep_rate_kmhs`, `stop_brake_offset_pct`, `engine_brake_decel_kmhs`, `coast_decel_speeds_kmh`, `coast_decel_kmhs`, `creep_accel_speeds_kmh`, `creep_accel_kmhs`, `pedal_gain_speeds_kmh`, `accel_gain_kmhs_per_pct`, `brake_gain_kmhs_per_pct` | 14 | `candidate`, `ref_smooth_sigma_s`, `ref_smooth_max_kmh`, `ref_smooth_brake_from_kmhs`, `ref_smooth_brake_to_kmhs`, `ref_smooth_brake_side`, `coast_band_kmhs`, `reach_horizons_s`, `reach_step_s`, `pedal_select_center_s`, `pedal_select_width_s`, `pedal_select_mode`, `pedal_select_point_s`, `pedal_select_brake_center_s` |
| features | なし | 1 | `h0_s` |
| forward_map | なし | 134 | `coast_band_kmhs`, `coast_band_release_at_zero`, `coast_fill_max_s`, `coast_fill_same_pedal`, `need_window_s`, `accel_lag_s`, `brake_lag_s`, `preposition_enabled`, `preposition_margin_pct`, `preposition_lead_s`, `deviation_c`, `deviation_row`, `dob_tq_coef`, `dob_fast_enabled`, `dob_fast_tq_coef`, `dob_fast_need_kmhs`, `coast_on_kmh`, `coast_off_kmh`, `coast_hold_coef`, `coast_floor_onset`, `coast_one_pedal_per_section`, `coast_release_on_cross`, `predictor_enabled`, `speed_filter_q`, `speed_filter_r`, `speed_filter_r_floor`, `speed_filter_accel_sigma`, `creep_floor_lead_s`, `stop_lead_enabled`, `stop_end_p_enabled`, `stop_end_accel_pred_enabled`, `stop_end_tau_cap_normal`, `dob_low_speed_fade`, `dob_low_speed_align`, `dob_split_low_speed`, `dob_split_blend_lo_kmh`, `dob_split_blend_hi_kmh`, `low_speed_side_rule`, `launch_midway_brake_cap`, `launch_release_ramp`, `launch_release_lead_s`, `launch_release_pre_pct`, `stop_tail_ramp_enabled`, `stop_tail_kmh`, `stop_approach_no_release`, `stop_approach_no_release_kmh`, `stop_approach_release_dev_kmh`, `stop_approach_step_release`, `stop_approach_creep_gate`, `launch_short_stop_s`, `launch_move_hold`, `fb_no_breakaway_floor`, `fb_no_deeper_at_ref_zero`, `fb_breakaway_band_cap`, `stop_end_brake_monotone`, `switch_gap_compensate`, `switch_gap_min_loss`, `switch_gap_rule`, `switch_gap_peak_from_corner`, `switch_gap_peak_robust`, `switch_gap_valley_robust`, `switch_gap_choose`, `switch_gap_min_peak`, `switch_gap_peak_window_s`, `pedal_track_accel_delay_s`, `pedal_track_accel_max_rate_pct_s`, `pedal_track_accel_max_accel_pct_s2`, `pedal_track_brake_delay_s`, `pedal_track_brake_max_rate_pct_s`, `pedal_track_brake_max_accel_pct_s2`, `pedal_track_accel_effect_lag_s`, `pedal_track_brake_effect_lag_s`, `accel_release_in_range_only`, `predict_low_speed_sides`, `launch_shift_choose`, `launch_gap_tight`, `launch_brake_band`, `launch_band_step_fb`, `launch_breakaway_observed`, `launch_band_shift`, `launch_band_late_release`, `c10_play_floor`, `play_floor_rise_kmhs`, `coast_assist_handover`, `coast_assist_pred_p`, `coast_assist_accel_no_floor`, `coast_assist_brake_wait_floor`, `coast_assist_accel_no_play`, `coast_assist_accel_unpark_pct`, `coast_fill_brake_wait_floor`, `predict_creep_curve`, `launch_midway_fb_no_deeper`, `launch_table_breakaway_floor`, `hold_pi_enabled`, `hold_ff_enabled`, `hold_ff_offset_pct`, `hold_pi_deadband_kmh`, `hold_lead_in_enabled`, `hold_accel_pred_enabled`, `switch_lead_enabled`, `release_table_min_kmh`, `low_speed_release_stop_layer`, `stop_approach_keep_stop_table`, `side_table_taper`, `low_speed_release_anchor`, `low_speed_switch_ramp`, `local_gain_brake`, `local_gain_accel`, `c10_release_table`, `c10_integral_brake`, `c10_integral_accel`, `c10_integral_ti_coef`, `c10_integral_low_speed`, `c10_integral_coast_assist`, `c10_small_dev_band`, `c10_small_dev_e_kmh`, `c10_small_dev_r`, `c10_d_term`, `c10_d_window_s`, `c10_d_accel_source`, `c10_d_filter_s`, `c10_d_td_coef`, `c10_p_local_gain`, `c10_p_local_gain_c`, `c10_p_local_gain_cruise_kmhs`, `c10_p_local_gain_taper_kmhs`, `c10_p_lag_s`, `c10_p_lag_min_kmh`, `c10_p_local_gain_source`, `c10_p_local_gain_gm_db`, `ff_hf_div`, `ff_hf_sigma_s`, `ff_hf_div_coef`, `ff_hf_fast_max_pct` |
| fel | なし | 22 | `enabled`, `method`, `c_max_kmhs`, `min_cell_s`, `speed_edges_kmh`, `accel_u_bin_pct`, `brake_u_bin_pct`, `stop_hold_s`, `ease_start_kmh`, `stop_end_window_s`, `low_trip_max_kmh`, `low_trip_count`, `cruise_s`, `mid_step_kmh`, `mid_round_trips`, `mid_speed_min_kmh`, `high_speed_quantile`, `high_coast_s`, `half_g_s`, `coast_s`, `accel_min_g`, `pattern_dt_s` |
| pid | なし | 4 | `kp`, `ki`, `kd`, `output_limit_pct` |
| arbiter | なし | 19 | `enable_deadband_compensation`, `enable_rate_limit`, `enable_hysteresis`, `switch_hysteresis_pct`, `accel_rate_limit_pct_s`, `brake_rate_limit_pct_s`, `accel_reengage_dwell_s`, `accel_release_rate_pct_s`, `accel_min_step_pct`, `enable_min_step`, `enable_reengage_dwell`, `enable_release_rate`, `enable_accel_band`, `accel_band_horizon_s`, `accel_band_kmhs`, `accel_band_dev_escape_kmh`, `accel_band_open_escape_pct`, `enable_direction_hysteresis`, `accel_direction_hysteresis_pct` |
| control | なし | 3 | `loop_interval_ms`, `log_interval_ms`, `can_max_speed_age_s` |
| modes | なし | 7 | `wltp_mode_name`, `tuning_mode_name`, `coverage_mode_names`, `segment_names`, `segment_bounds_s`, `segmented_mode_names`, `composed_modes` |
| mode_drive | なし | 4 | `decel_governor`, `governor_reduce_step_pct`, `pedal_standby`, `pedal_interlock_enabled` |
| excite | なし | 27 | `speeds_kmh`, `frequencies_hz`, `amplitude_pct`, `hold_s`, `min_periods`, `analysis_skip_s`, `approach_timeout_s`, `approach_band_kmh`, `approach_settle_s`, `approach_kp_pct_per_kmh`, `approach_ki_pct_per_kmh_s`, `approach_max_rate_pct_s`, `approach_initial_offset_pct`, `trim_gain_pct_per_kmh_s`, `speed_lpf_tau_s`, `abort_band_kmh`, `brake_enabled`, `brake_frequencies_hz`, `brake_amplitude_pct`, `brake_extra_decel_kmhs`, `brake_start_offset_kmh`, `brake_coast_s`, `brake_ramp_pct_s`, `brake_ramp_max_pct`, `brake_response_lag_s`, `brake_band_margin_kmh`, `brake_decel_cutoff_ratio` |
| tuning | なし | 2 | `max_runs`, `select_metric` |
| learning | なし | 92 | `timeout_s`, `coast_timeout_s`, `ramp_g_levels`, `ramp_mode_min_s`, `ramp_top_margin_kmh`, `ramp_time_budget_s`, `ramp_max_passes`, `ramp_cruise_band_g`, `ramp_cruise_hold_s`, `ramp_cruise_timeout_s`, `ramp_cruise_fine_max_kmh`, `ramp_cruise_coarse_step_kmh`, `ramp_cruise_sine_enabled`, `ramp_cruise_sine_hz`, `ramp_cruise_sine_amplitude_pct`, `ramp_cruise_sine_periods`, `ramp_release_fracs`, `ramp_release_move_s`, `ramp_release_hold_s`, `ramp_release_enabled`, `ramp_release_sweep_enabled`, `ramp_release_sweep_rate_pct_s`, `ramp_release_sweep_time_s`, `ramp_release_sweep_timeout_s`, `ramp_release_sweep_est_s`, `ramp_release_sweep_levels`, `ramp_lag_speeds_kmh`, `ramp_lag_step_g`, `ramp_lag_step_hold_s`, `ramp_lag_approach_g`, `ramp_settle_time_s`, `ramp_launch_time_s`, `ramp_turn_time_s`, `ramp_stop_time_s`, `ramp_edge_g`, `ramp_launch_fast_rate_pctps`, `ramp_launch_release_margin_pct`, `ramp_launch_slow_rates_pctps`, `ramp_hold_time_s`, `ramp_min_at_g_s`, `ramp_launch_g_rate_gps`, `ramp_launch_blend_min_s`, `ramp_one_sided`, `ramp_dev_deadband_kmh`, `ramp_gprop_without_sensitivity`, `ramp_actuator_lag_tol_pct`, `gov_soft_frac`, `accel_gain_min_offset_pct`, `brake_gain_min_offset_pct`, `creep_launch_settle_kmhs`, `creep_launch_settle_s`, `creep_curve_bin_kmh`, `creep_curve_min_bin_samples`, `coast_curve_low_bin_kmh`, `coast_curve_low_max_kmh`, `coast_curve_low_min_bin_samples`, `grid_settle_tol_kmh`, `fel_pattern_enabled`, `side_table_opening_source`, `launch_breakaway_move_kmh`, `launch_accel_side_source`, `side_table_accel_window_s`, `launch_rows_to_stop_max`, `side_row_speed_rep`, `stop_release_layer_enabled`, `accel_release_input_enabled`, `accel_push_input_enabled`, `stop_release_enabled`, `stop_release_start_kmh`, `stop_release_fracs`, `stop_release_rate_pct_s`, `stop_release_escape_s`, `stop_release_escape_decel_kmhs`, `stop_release_max_g`, `stop_approach_enabled`, `stop_approach_g_levels`, `stop_approach_counts`, `stop_approach_accel_g`, `stop_approach_hold_s`, `stop_approach_settle_s`, `stop_approach_max_g`, `stop_table_sub_min_weight`, `launch_table_sub_min_weight`, `stop_approach_exclusion`, `ramp_dither_enabled`, `ramp_dither_brake_speeds_kmh`, `ramp_dither_brake_u0_from_onset_pct`, `ramp_dither_deltas_pct`, `ramp_dither_hold_s`, `ramp_dither_max_g`, `ramp_dither_accel_band_pct`, `ramp_dither_accel_max_n` |
| pedal_search | なし | 21 | `step_mm`, `dwell_s`, `onset_margin_kmh`, `creep_window_s`, `creep_settle_kmhs`, `creep_settle_min_s`, `creep_min_speed_kmh`, `creep_timeout_s`, `stop_hold_margin_pct`, `stop_confirm_max_wait_s`, `brake_step_mm`, `low_stair_enabled`, `low_stair_step_pct`, `low_stair_hold_s`, `low_stair_max_steps`, `low_stair_press_max_steps`, `low_stair_repeats`, `low_stair_creep_margin_kmh`, `hold_points_enabled`, `hold_points_fractions`, `hold_points_extra_kmh` |
| decel_stop | なし | 8 | `target_decel_g`, `press_margin_g`, `release_above_g`, `step_mm`, `dwell_s`, `slope_window_s`, `approach_margin_pct`, `timeout_s` |
| kpi | なし | 16 | `max_abs_deviation_kmh`, `p95_deviation_kmh`, `reversal_band_kmh`, `reversal_window_s`, `reversal_limit_per_window`, `pedal_reversal_hyst_pct`, `pedal_reversal_limit_per_s`, `pedal_reversal_window_s`, `pedal_reversal_window_limit_per_s`, `pedal_reversal_min_window_s`, `pedal_overlap_margin_pct`, `iwr_min_pct`, `iwr_max_pct`, `drive_index_iwr_min_pct`, `drive_index_iwr_max_pct`, `drive_index_rmsse_max_kmh` |
| checks | なし | 13 | `init_servo_comm`, `init_clear_errors`, `init_servo_on`, `init_can`, `init_ups`, `init_home_return`, `pre_communication`, `pre_servo_state`, `pre_profile`, `pre_ups`, `pre_actuator_position`, `pre_brake_stop`, `pre_vehicle_stopped` |
| buttons | **全部**（15 個）＋ buttons.enabled（src 専用）: `start_short_press_s`, `start_long_press_s`, `shift_press_s`, `after_press_wait_s`, `accel_tap_opening_pct`, `accel_tap_on_s`, `accel_tap_off_s`, `maintenance_tap_count`, `n_hold_gap_s`, `d_press_at_s`, `no_speed_abort_s`, `no_speed_kmh`, `servo_rest_angle_deg`, `servo_map` | 0 | なし |
| output | なし | 4 | `results_dir`, `csv_interval_s`, `plot`, `print_interval_s` |
| hardware | なし | 2 | `settings_path`, `database_url` |

`feedforward` の車両キーは手順 2（`pattern_drive.FF_PARAM_KEYS` から `MEASURED_KEYS` を除いたもの + `model_path`）・手順 2-0（`pedal_search.config_updates` の `stop_brake_offset_pct`・`creep_speed_kmh`）が `cfg.save()` で書くもの。ほかに `VEHICLE_FEEDFORWARD_KEYS` には多項式モデル（C1〜C5。`_build_poly_model`）だけが書く `model_accel/brake_horizons_s`・`model_accel/brake_features`・`model_coef_path` の 5 個も入れてある。これらは今の core の `FeedforwardSection` にフィールドが無く、tests でも保存で失敗する（C8 などの今の経路では呼ばれない）ため、車両 yaml には書いていない。`pid.kp` は手順 0 が同じ値を書き戻すだけ（共通。値が変わる保存は断る）。`buttons.enabled` は src 専用で、core に渡す前に合成 yaml から取り除く。

dataclass にあって yaml に書かれていないキー（既定値が効く。tests の yaml も同じ）は 20 個程度ある（`forward_map.launch_shift_center_lag`・`forward_map.low_speed_keep_*`・`learning.stop_brake_floor_*` など）。振り分けは yaml に書かれているキーだけで、これらは共通側の扱い（車両キーに入れない）。

#### C-8「試験車で合わせた値」の一覧

共通側に入っているキーのうち、yaml の注記に「実機」の走行で採用・決定・測定した旨が書かれているもの（64 個。`deviation_c`・`ff_hf_div` を含む）。別の車で走るときの見直し対象。注記は `config/robot_common.yaml` のキー直前のコメント（行範囲）。根拠の走行（`ProblemReport_20261109` の PDCA・段）はその注記に書いてある。この一覧は注記の語句から機械的に拾ったので、実際には車に依らない固定値と判断できるものが混じっている可能性がある（見直しの優先度は別途ユーザーと決める）。

| キー | 今の値 | 注記の位置（config/robot_common.yaml の行） |
|---|---|---|
| `feedforward.ref_smooth_sigma_s` | `0.15` | 46〜53 |
| `feedforward.ref_smooth_brake_side` | `upper` | 62〜64 |
| `feedforward.pedal_select_brake_center_s` | `0.0` | 89〜94 |
| `forward_map.coast_band_release_at_zero` | `True` | 109〜117 |
| `forward_map.coast_fill_max_s` | `2.0` | 118〜128 |
| `forward_map.deviation_c` | `6.0` | 153〜164 |
| `forward_map.deviation_row` | `reference` | 165〜167 |
| `forward_map.dob_tq_coef` | `20.0` | 168〜174 |
| `forward_map.coast_on_kmh` | `0.2` | 183〜191 |
| `forward_map.stop_lead_enabled` | `True` | 232〜244 |
| `forward_map.stop_end_accel_pred_enabled` | `False` | 246〜254 |
| `forward_map.launch_midway_brake_cap` | `True` | 293〜297 |
| `forward_map.launch_release_ramp` | `True` | 298〜303 |
| `forward_map.stop_tail_ramp_enabled` | `True` | 306〜312 |
| `forward_map.stop_approach_no_release` | `True` | 316〜328 |
| `forward_map.switch_gap_peak_robust` | `True` | 383〜387 |
| `forward_map.switch_gap_valley_robust` | `True` | 388〜393 |
| `forward_map.switch_gap_min_peak` | `False` | 399〜404 |
| `forward_map.predict_low_speed_sides` | `True` | 428〜433 |
| `forward_map.launch_band_step_fb` | `True` | 457〜463 |
| `forward_map.launch_breakaway_observed` | `True` | 464〜470 |
| `forward_map.launch_band_shift` | `True` | 471〜473 |
| `forward_map.launch_band_late_release` | `True` | 474〜480 |
| `forward_map.c10_play_floor` | `True` | 481〜488 |
| `forward_map.coast_assist_pred_p` | `False` | 497〜500 |
| `forward_map.predict_creep_curve` | `True` | 522〜526 |
| `forward_map.hold_pi_enabled` | `True` | 537〜542 |
| `forward_map.low_speed_release_stop_layer` | `False` | 568〜574 |
| `forward_map.stop_approach_keep_stop_table` | `False` | 575〜579 |
| `forward_map.side_table_taper` | `True` | 580〜583 |
| `forward_map.local_gain_brake` | `False` | 590〜596 |
| `forward_map.c10_release_table` | `False` | 598〜602 |
| `forward_map.c10_integral_low_speed` | `off` | 612〜617 |
| `forward_map.c10_small_dev_band` | `False` | 621〜625 |
| `forward_map.c10_d_term` | `False` | 628〜631 |
| `forward_map.c10_d_accel_source` | `window` | 633〜639 |
| `forward_map.c10_p_local_gain` | `cruise` | 642〜649 |
| `forward_map.c10_p_local_gain_source` | `hf` | 657〜662 |
| `forward_map.ff_hf_div` | `cruise` | 665〜673 |
| `fel.enabled` | `False` | 685〜686 |
| `arbiter.accel_min_step_pct` | `1.0` | 737〜740 |
| `arbiter.accel_band_open_escape_pct` | `100.0` | 751〜751 |
| `arbiter.accel_direction_hysteresis_pct` | `0.5` | 753〜753 |
| `excite.brake_enabled` | `True` | 846〜857 |
| `learning.ramp_g_levels` | `[0.02, 0.045, 0.08, 0.125, 0.175, 0.2...` | 887〜890 |
| `learning.ramp_time_budget_s` | `1600.0` | 896〜905 |
| `learning.ramp_cruise_band_g` | `0.005` | 911〜919 |
| `learning.ramp_cruise_hold_s` | `5.0` | 920〜930 |
| `learning.ramp_release_fracs` | `[0.5, 0.0]` | 950〜959 |
| `learning.ramp_release_sweep_enabled` | `False` | 968〜977 |
| `learning.ramp_lag_speeds_kmh` | `[30, 60]` | 1002〜1012 |
| `learning.ramp_settle_time_s` | `2.0` | 1016〜1020 |
| `learning.ramp_launch_time_s` | `1.0` | 1021〜1029 |
| `learning.ramp_hold_time_s` | `7.0` | 1048〜1051 |
| `learning.ramp_min_at_g_s` | `3.25` | 1052〜1059 |
| `learning.ramp_launch_g_rate_gps` | `0.15` | 1060〜1068 |
| `learning.ramp_one_sided` | `True` | 1070〜1079 |
| `learning.ramp_dev_deadband_kmh` | `0.5` | 1080〜1084 |
| `learning.accel_push_input_enabled` | `False` | 1163〜1169 |
| `learning.stop_table_sub_min_weight` | `True` | 1203〜1204 |
| `learning.launch_table_sub_min_weight` | `True` | 1205〜1206 |
| `pedal_search.low_stair_enabled` | `False` | 1269〜1274 |
| `pedal_search.hold_points_fractions` | `[0.25, 0.5, 0.75]` | 1297〜1299 |
| `pedal_search.hold_points_extra_kmh` | `[1.6]` | 1300〜1303 |

#### 等価確認①の結果

`.venv/bin/python scripts/check_config_equivalence.py`（tests の `load_config(config_testVehicle.yaml)` と、合成した `ResearchConfig` を `dataclasses.asdict` で全フィールド比較）:

- 比べたキー 448 個（置き場所のキー 5 個を除く 443 個）→ **不一致 0 個**
- 置き場所のキー（`output.results_dir`・`output.plot`・`feedforward.model_path`・`model_coef_path`・`vehicle.road_load_file`）の確認: OK（model_path は車両の `models/` の下で tests と同じファイル名・pkl が存在・results_dir は `.work`・plot は false・road_load_file は null で走行抵抗は tests が private から読んだ値と一致）
- `validate_config` の結果も同じ・`stop_brake_opening_pct` も同じ
- 保存の往復（試験車の写しに、手順 2 が書くのと同じキーを `cfg.save` で書いて同期）: 変わったのは書いた 2 キー（`model_path`・`creep_speed_kmh`）の行だけで、コメントは保たれた

### 段2（2026-11-15）: ランナー

作ったもの（詳細・イベント形式の表・差し替え点の表は `src/robot/README.md`）:

| ファイル | 内容 |
|---|---|
| `src/robot/run.py` | `python -m src.robot.run --vehicle 車ID --steps 1,3｜--interactive [--hw] [--mode] [--limit-s] [--vehicles-dir] [--common] [--events-fd]`。core の `run_steps` を呼ぶだけ（終了コードも core と同じ。足したのは中断の 130/143）。`cfg.save` の後に車両 yaml へ写す（`install_save_sync`）。動かせる手順は 1・2・3・12〜14・20〜25。`buttons.enabled` が false の車で 20〜25 は終了コード 2 |
| `src/robot/events.py` | イベント（JSON 1 行。標準出力は `@@EV ` 前置き・`--events-fd` なら fd）: `state`・`init_step`・`sample`（5 Hz）・`csv`（kind = learning/drive）・`result`・対話用の `ack`・`status` |
| `src/robot/streaming_log.py` | `StreamingSessionLog`（`SessionLog` の派生。record で sample、close で csv） |
| `src/robot/patches.py` | 差し替え点を 1 か所に集約（下） |
| `src/robot/interrupt.py` | SIGINT/SIGTERM。手順の本体だけを cancel して core の finally（終了処理）へ進ませる |
| `src/robot/interactive.py` | 対話ジョブ: jog・move・home・status・save_min・quit。max で頭打ち・片足ルール（もう一方が min 超なら先に min へ戻す）。save_min は `config_store.save_vehicle_values` で車両 yaml（コメントは保つ） |
| `scripts/check_runner_equivalence.py` / `check_runner_interrupt.py` | 等価確認②・中断の確認 |
| `tests/unit/test_robot_runner.py` | イベント・引数・対話・中断・差し替えのテスト（41 件） |

差し替え点（core は直していない）: ①`SessionLog` → `StreamingSessionLog`（main・pattern_drive・pre_drive_check のモジュール属性）②`main.write_mode_report` → 何もしない（A-2: レポート md・図は作らない）③`InitReport.add/skip` を包む（init_step。say の行はそのまま出る）④`main.pre_drive_check`・`main.shutdown` を包む（state・走行前チェックを中断可能に）。ほかに各 `Step` を `dataclasses.replace` で包む・`--mode` は `cfg.modes.wltp_mode_name` と合成 yaml の同キーを書く（学習の手順とは同じ実行にできない）。

設定の変更: `output.results_dir` を `.work` から `vehicles/<車ID>/data/` に変更（合成 yaml は `.work` のまま）。等価確認①は引き続き一致（不一致 0）。`import_research_vehicle.py --data-only` を追加（pkl の meta `sources` の学習 CSV を data/ へコピー）。

#### 等価確認②（スタブ・`--steps 1,3 --limit-s 60`、モード走行 1200 行を mode_time_s で突き合わせ）

| 比べた組 | accel_cmd 最大/p95 差 [%] | brake_cmd 最大/p95 差 [%] | 車速 最大/p95 差 [km/h] | 基準車速 最大/p95 差 |
|---|---|---|---|---|
| tests 同士（ゆらぎ） | 1.499 / 0.007 | 0.004 / 0.001 | 0.015 / 0.003 | 0.012 / 0.002 |
| tests – src（4 組の範囲） | 0.96〜1.49 / 0.002〜0.007 | 0.003〜0.004 / 0.001〜0.002 | 0.012〜0.015 / 0.002〜0.003 | 0.006〜0.013 / 0.002 |

行数は全区間で同じ（MODE_DRIVE 1200・DECEL_TO_STOP 417。PRE_DRIVE_CHECK は 60 と 61）。tests 2 本・src 2 本。判断: 差は tests 同士の時刻のゆらぎと同じ大きさで、ゆらぎの範囲（1.5 倍＋余裕）に収まる（最大差 1 %台は 1 周期だけ出る accel の跳びで、tests 同士でも同じ）。**等価**。

#### そのほかの確認（スタブ）

- 手順 2（`--steps 1,2`、2308 s）: 手順 2-0 〜 G ランプ〜モデル作成まで最後まで走った。最後に ConfigError（`forward_map.launch_brake_band` は pkl の meta に hold_pi・hold_opening が要る。スタブでは 4 点の保持が 1 点も保てない）で終了コード 2。車両 yaml は変わらず（md5 一致）。`cfg.save` は呼ばれなかった。スタブが吐いた pkl は削除。**tests 側で同じ手順 2 を走らせての比較はしていない**（38 分かかるため。core の ConfigError で、ランナーの差し替えが関与する場所ではない）。
- 手順 24（`--steps 1,24 --limit-s 60`、buttons.enabled true）: ボタンのスタブは動く（Ready・D・N を押す）が、スタブ車両が D の後に車速を出さず「3 s 車速が出ない」中止（終了コード 5）。**tests の `main --steps 1,24` も同じ時刻で同じ中止**（スタブ車両の制約）。
- 対話ジョブ（スクリプトから）: status→jog→jog（片足ルールでアクセルを先に min へ）→move 99（max に頭打ち）→不正コマンド→home→quit が期待どおり。save_min は車両 ID の一時コピーで確認（test_vehicle の値は変更なし）。min 5.5% は pulse の丸めで 5.49 になる。
- 引数エラー（動かせない手順・buttons 無効の車の 20〜25・`--mode` の誤用・存在しない車/モード）はすべて終了コード 2・HW に触らない。`--mode` は DB の ID でも名前でも引ける。

#### 中断（SIGINT/SIGTERM。スタブ・走行中）

| 送る場面 | シグナル | 終了まで | 終了コード | 終了処理 |
|---|---|---:|---:|---|
| モード走行の 5 秒後 | SIGINT（0.5 s 後に 2 回目を送ったが、先に終了） | 0.39 s | 130 | 「終了処理: 完了」あり・CSV 167 行を保存 |
| モード走行の 5 秒後 | SIGTERM | 0.40 s | 143 | 同上（166 行） |
| 手順の準備（同期の重い計算の最中）の 10 秒後 | SIGINT ＋ 0.05 s 後に 2 回目 | 0.03 s | 130 | あり。2 回目は無視（ログに出る） |

準備（WLTP で約 2 分の同期計算）がメインのイベントループを止めるため、最初は準備中のシグナルが 2 分後まで処理されなかった。準備だけ別スレッドへ逃がして解決（中断したらスレッドは見捨てて `os._exit`。準備は HW に触らない）。

#### 残った問題

- **学習 CSV が無い**: 試験車の pkl の meta `sources` の `drive_log_real_20261029_062805.csv` が tests/research/results にも無く、`--data-only` でコピーできなかった。手順 13・14 を試験車で動かすには、このログを戻すか、手順 2 をやり直す。
- 手順 2 のスタブは最後のチェックで ConfigError になる（上）。実機の手順 2 は段8 で通す。
- 手順 2 のモデル作成（`asyncio.to_thread`）の最中に中断すると、そのスレッドが終わるまで（数十秒〜）プロセスが終わらない。ペダルは先に原点へ戻っている。
- ランナーは `run_steps` が出す「次は手順 4 です」などの core の文言をそのまま流す（Web 側は state イベントで状態を判断する）。
- `--limit-s` 付きの 1 本は準備 120 s ＋ 走行前チェック ＋ 走行 60 s ＋ 停止 20 s で約 205 s かかる（tests も同じ）。

### 段3（2026-11-15）: Web 側のジョブ管理

詳細（ジョブの種類・セッションの判断・CSV の列の対応・API の残す/外す/足す表）は `src/robot/README.md`「Web 側（段3・段4）」。

| ファイル | 内容 |
|---|---|
| `src/app/job_manager.py` | `JobManager`: ランナーを asyncio のサブプロセスで 1 本ずつ起動（同時 1 本・実行中は 409）。標準出力の `@@EV ` は状態へ、それ以外は進行のメッセージ（`LogBufferHandler` ＋ DB `drive_session_messages` へ 1 秒ごとにまとめ書き）。状態は待機/実行中/非常停止中。停止・非常停止は SIGINT → `KILL_TIMEOUT_S`（40 s）で SIGKILL。アプリ終了時も止めて待つ。対話ジョブへのコマンド送信（ack/status 待ち）。終了時に CSV を取り込み（drive の CSV は消す）、セッションを completed/error/emergency にする |
| `src/infra/drive_recorder.py` | 記録先（`PgDriveRecorder`: セッション・メッセージ・`copy_records_to_table` で 5000 行ずつ CSV 取り込み／`NullDriveRecorder`: DB なし） |
| `src/infra/drive_csv.py` | CSV の 9 列 ⇔ drive_logs ＋ `detail JSONB` の相互変換（純粋）。列順は core の `CSV_COLUMNS` |
| `src/app/inmemory_repos.py` | DB なし用の in-memory リポジトリ・UPS スタブ（古い `stubs.py` を Web が import しないよう写した） |
| `src/web/app.py` | lifespan から `RobotController`・学習サイクル・`build_*_controller` を外し、`JobManager`・`VehicleStore` を `app.state` に。実機/ベンチでは GPIO・UPS の非常停止を `JobManager.emergency` へ |
| `src/web/routers/drive.py`・`ws.py`・`deps.py`・`schemas.py` | 上の API 表のとおり。`RealtimeData.cycle_progress` → `job`、電流は 0.0 固定 |
| `scripts/setup_db.py` | 冪等な追加（下） |

**DB の変更**（`scripts/setup_db.py`。本番 DB に 2 回実行して冪等を確認。既存の行は変更なし: drive_sessions 56・drive_logs 97281 のまま）: `drive_sessions.vehicle_id TEXT`・`profile_id` を NULL 可・索引 `idx_drive_sessions_vehicle_id`、`drive_logs.detail JSONB`、表 `drive_session_messages`（id・session_id FK ON DELETE CASCADE・seq・elapsed_s・time・level・message）＋索引 `(session_id, seq)`。

**sessions API**: 一覧に `vehicle_id`・`vehicle_name`、`GET /sessions/{id}/messages`、`/logs.csv` は detail を展開して core の `CSV_COLUMNS`（83 列）の順で全列を DB のカーソルから流して返す。JSON の `/logs` は detail を読まない（9 列だけ）。

### 段4（2026-11-15）: 車両 API

| ファイル | 内容 |
|---|---|
| `src/infra/vehicle_store.py` | `VehicleStore`: 一覧・読み・作成（雛形 `TEMPLATE`）・人が決める値の更新・削除・選んでいる車（`vehicles/.active`）。書き込みは core の `update_yaml_values`（コメント保持）。書く前に共通設定と合成して `validate_config`（落ちたら `VehicleInvalid` → 422 に日本語の理由、何も書かない）。min の「測定済み」は `vehicles/<ID>/state.json`（新規作成は未測定・state.json の無い車は測定済み。save-min・min の更新で true） |
| `src/web/routers/profiles.py` | `/api/v1/profiles/*` を車両 yaml に置き換え（DB の profiles を読むコードは外した。DB の行は消していない） |

### 確認の結果（段3・段4、スタブ）

| 項目 | 結果 |
|---|---|
| ruff check（src・scripts・足したテスト） | 通過 |
| 足した単体テスト | `test_job_manager.py` 20 件（偽ランナーでサブプロセスごと: 完了・記録・CSV の取り込み/削除/残し・取り込み失敗・SIGINT・非常停止の解除手順・SIGKILL・対話・アプリ終了）、`test_drive_csv.py` 8 件（9 列の対応・往復）、`test_vehicle_store.py` 11 件。計 39 件通過（4.4 s＋5.5 s＋1.5 s） |
| uvicorn（スタブ・ポート 8081・DB は本番 DB の複製 `driving_robot_stage3`・車両は scratchpad の複製。終了後に停止・DB 削除） | 下 |
| 初期化 | 車を選ばずに `/initialize` → 409（日本語）。選んで `/initialize` → INITIALIZING → 1.5 s 以内に READY・終了コード 0・`init_steps` 8 項目（7 done・1 skipped） |
| モード走行（09_US06・`limit_s` 60・手順 1,3） | `/start` → 200（session_id あり）。実行中の 2 本目の `/start` → 409。149.5 s で終了コード 0 → `drive_sessions` completed（vehicle_id＝stub_car・profile_id NULL・run_type auto）。drive_logs 1836 行（MODE_DRIVE 1200・DECEL_TO_STOP 575・PRE_DRIVE_CHECK 61。段2 の MODE_DRIVE 1200 と同じ）・detail は全行・キーの種類 56。進行のメッセージ 265 行（seq 1〜265・経過秒つき）。CSV はディスクから消えた。`/logs.csv` は 1836 行・83 列で見出しが core の `CSV_COLUMNS` と完全一致・全行 83 セル |
| 非常停止 | 走行 13 s の最中に `/emergency` → 約 0.5 s でランナーが「終了処理: 原点復帰 → サーボOFF → 切断 → 完了」を出して終了コード 130。セッション emergency・drive_logs 326 行を取り込み済み。終了後も EMERGENCY のまま・`/start` → 409。`/reset-emergency` → 200 → READY |
| 対話ジョブ | calib/jog（pulse→%換算）・calib/home・calib/save-min（車両 yaml の min_brake を書き、測定済みに）・手動中の calib は 409（目的が違う）・`/stop` で終了。manual/start（セッション manual）→ jog → home → stop → completed。calib はセッション無し |
| 学習 | `/learning/start {kind: full}` → セッション learning・ジョブ中の車の削除/更新は 409・`/learning-cycle/abort` → 終了コード 130・セッション error |
| 車両 API | 作成（名前から ID。日本語だけの名前は `vehicle_日時`、同名は `_2`）・更新（コメント保持）・範囲外（max_speed 999）→ 422 `vehicle.max_speed_kmh が範囲外: 999.0`・重複 ID → 409・削除 → 204・存在しない車 → 404。WebSocket: 待機は READY・キャリブレーション中は CALIBRATING・`job.interactive_status` が届く |

### 迷って決めたこと

- **選んでいる車**: サーバー側（`vehicles/.active`）に持ち、`select-profile` のパスを残した。`/initialize`・`/start` に車の指定が無いのと、複数の画面で同じ車にするため。
- **待機中の robot_state は READY**（どのジョブも自分で初期化するのでいつでも始められる）。旧画面のゲートがそのまま通る。
- **対話ジョブは目的（manual/calib）を固定**。違う目的で呼ぶと 409（手動はセッションを作るがキャリブレーションは作らないため）。キャリブレーションの終わりは `/drive/stop`（新しい API を足さない）。
- **`KILL_TIMEOUT_S` は 40 s**（計画の例は 10 s）。core の原点復帰が 1 軸 30 s までのタイムアウトで、それより短く SIGKILL すると原点復帰の途中でペダルを踏んだまま残るため。実機の所要時間は段8 で測って見直す（スタブは 0.4〜0.5 s）。
- **新しい車の雛形のクリープ加速カーブは仮の 2 点**（`[0.5, 5.0]`・`[1.0, 0.0]`）: core の `validate_config` が空のカーブを許さず、空だとランナーが設定エラー（終了コード 2）で手順 2 に進めないため。手順 2-0 が測り直して置き換える。他の学習の値（惰行・ペダルゲイン）は空、`model_path` は null。
- **セッションを作らないジョブ**: キャリブレーション・init_only（残す値が無い）。DB に繋がらないときはセッションを作れず、走行・学習・手動の開始は 503（記録が DB だけなので、記録できないまま走らせない）。
- **学習の CSV（kind=learning）は取り込むが消さない**（手順 13・14 が `data/` から読み直す）。drive の CSV は取り込み成功後だけ消す（失敗したら残す）。
- 古い単体テスト `tests/unit/test_web_drive.py`（50 件）・`test_ws_broadcast.py`（6 件）は旧 `RobotController` の API を試すもので失敗する（変更禁止のため触っていない）。段7 で古いコードと一緒に消す。

### 残った問題

- 画面（`src/web/static/js`）は旧 API のまま。profiles/calibration/learning/auto-drive は段5 で直す（旧 PID/FF 欄・ゼロ/フル点・一時停止・プランの呼び先は 404 になる）。ログ画面のメッセージ表示も段5。
- タイムスケジュール走行（`/drive/schedule/start`）は 501（段6）。
- `buttons.servo_map`（ch・角度）は読むだけで、API では書けない（`update_yaml_values` が入れ子の 1 行値を書けないため）。サーボの角度合わせの画面が要るなら別途。
- 実機・ベンチの経路（GPIO の非常停止 → `JobManager.emergency`、UPS の AC 断、`--hw real` の起動）は書いたが、スタブでは動かしていない（`DRIVING_ROBOT_USE_REAL_HW=1` は使っていない）。段8 で確かめる。
- 非常停止の「原点へ戻るまでの秒数」はスタブの 0.5 s だけ。実機は段8。
- WebSocket の電流（`accel_current_ma`・`brake_current_ma`）は 0.0 固定（ランナーの sample に電流が無い）。必要なら sample に足す。
- `drive_logs.detail` は文字列のまま（約 56 キー × 数万行）。1 走行の DB 取り込みは 1836 行で 1 s 未満だったが、WLTP 全体（約 3 万行）の所要時間・DB 容量は段8 の実走行で見る。


### 段5（2026-11-15）: 画面

画面は今の見た目・部品（Box・Btn・Pill・Row・DriveMonitorScreen・JogRow など）のまま、合わない所だけ作り替えた。

| 画面（ファイル） | 変更 |
|---|---|
| 共通（`js/job-utils.js` 新規・`app.js`・`index.html`） | `describeApiError`（409/422/423/503/501/404 の detail を「何が起きたか＋次にすること」の 2 行にし、`apiFetch` のトーストを 9 秒に）、終了コードの意味表（README の表。2・4・5・7・8・130 など）、`JobResultBanner`（結果・理由・「やり直す」）、`useDriveLogLines`・`useLastJob`（終わったジョブは `GET /drive/status` の `last_job`）、学習の段階表示（`deriveStages`・`StageRail`） |
| ホーム（`screens/home.js` 新規・NAV 先頭・既定の画面） | 選んでいる車の準備状況 4 段（① 車を登録 ② min を測る→キャリブレーションへ ③ 学習→学習運転へ ④ 走行できる→自動運転へ）。`min_*_measured`・`learned` から判定 |
| プロファイル（`profiles.js` 作り直し） | 旧 PID・FF・ゼロ/フル欄を廃止。編集: 名前・最高速・最大減速G・max/min 開度・停止判定・ボタンの秒数/回数・`servo_map`（ch・角度の差・実角度の表示）・運転指標用の値。読むだけ: 学習済み・学習日時・pkl・candidate・feedforward 一覧。保存は**変えた値だけ**送る（min を送ると測定済みになるため）。一覧は min 測定済み/未測定・学習あり/なし、削除は画面内の確認、新規作成は作った車をそのまま選ぶ |
| ボタン角度（`vehicle_store.py`・`schemas.py`） | `rewrite_servo_map_text`: servo_map の 1 行（`start: {channel: 0, offset_deg: -58.0}`）の `{…}` の中だけ作り直し、行末コメントは保つ。`PUT /profiles/{id}` の `buttons.servo_map: {start|P|N|D: {channel?, offset_deg?}}`。core の検証（0〜180°・同じ ch 同じ角度の重複）に通らなければ何も書かない。core の `update_yaml_values` は変えていない |
| キャリブレーション（`calibration.js`・`calib-components.js`） | ゼロ/フル → min 測定。開始（対話ジョブ）→ ジョグ（0.1 / 1 の 2 段・キー s/d/w/e・ドラッグ）→「ここを min として保存」（`calib/save-min`）→ 終了（`/drive/stop`）。開度 %・保存済み min・測定済み印を表示 |
| 学習運転（`learning.js`） | 開始の種類（学習／FEL 手順12／保持の点 13／停止の手前 14）。段階表示 準備 → ペダル探索 → G ランプ走行 → モデル作成 → 学習結果チェック → 補正表の学習。終了コード別の日本語（8＝yaml 書き換えなし・理由・やり直す、130＝中止 など）。中断・非常停止ボタン |
| 自動運転（`auto-drive.js`） | 一時停止・プラン・arm/cancel を削除。ボタン操作ありの車は「手順24」・なしは「手順3」と表示。中止・非常停止。スケジュール/シーケンスの開始は「準備中」で押せない。開発用: URL `?dev_limit_s=60` で API の `limit_s`（画面に開発用と表示） |
| 初期化・非常停止（`init.js`・`system-states.js`・`sketch.js`） | 初期化はジョブとして進み、終わると直前ジョブの項目と結果を表示。非常停止は実際のジョブ状態（終了処理中は解除ボタン無効）。解除ボタンを隠していたオーバーレイを外した |
| ログ（`logs.js`） | 行に車の名前、図の下に進行のメッセージ（等幅・経過秒・WARNING 黄・ERROR 赤）、CSV は全列、表は先頭 1000 行（全部は CSV） |
| 手動運転（`manual.js`） | `calibration.is_valid`・ZERO/FULL 換算・電流表示（0 固定）を外し、開度は pulse/9500 で換算 |

**確認（スタブ・ポート 8081・本番 DB の複製・車両は scratchpad の複製。終了後に停止・削除済み）**

| 項目 | 結果 |
|---|---|
| ruff check src scripts・単体テスト | 通過。`test_vehicle_servo_map.py` 7 件（行の書き換え・コメント保持・別ブロックの同名キー・検証 NG で何も書かない）＋既存 `test_vehicle_store`・`test_no_tests_import` 計 18 件 |
| コンソール | 全画面（ホーム・初期化・プロファイル・走行モード・スケジュール・シーケンス・キャリブ・学習・自動・手動・ログ）でエラー 0（favicon の 404 と、わざと起こした 422 のネットワーク行だけ） |
| 車の新規作成 → 編集 | 作成 → 自動で選択 → servo の D 差 85→80 と shift_press_s を保存。yaml は D 行と 1 行だけ変わりコメント保持・min は未測定のまま。max_speed 999 → 422 のトースト（理由＋次にすること） |
| キャリブレーション | 開始 → ジョグ → アクセル min 4.21 を保存 → 測定済み → ホームの ② に ✓ |
| 学習運転 | 開始 → 段階表示（準備 → ペダル探索）→ 中断 → 終了コード 130 の表示と「やり直す」 |
| 自動運転 | ボタンなし（手順3・US06・limit_s 60）で 1,3・終了コード 0・completed。ログ画面に図・進行のメッセージ 265 行・CSV（83 列・1833 行）。走行中の非常停止 → 解除 |
| スクリーンショット | `.playwright-mcp/s5_01〜s5_29`（ホーム・プロファイル・キャリブ・学習・自動・ログ・非常停止・エラー・初期化） |

**迷って決めたこと**: ホームは NAV に「ホーム」を足して既定の画面にした（今の画面にホームが無いため）。キャリブレーションの開始は最初の `calib/home` 呼び出し（無害）で対話ジョブを始める。学習の段階の「モデル作成」「チェック」は走行ログが無いので、進行のメッセージの行（`2-2.`・チェック表の見出し・`2-3.`）で見分ける（ジョブ開始時刻以降の行だけ。ブラウザとサーバーの時差が無い前提）。

**残った問題**
- ボタンあり（手順24）の車をスタブで走らせると、D を押しても入らず 3 秒で走行エラー（終了コード 5「D に入っていない可能性」）になった（画面は結果・理由・次の操作を出せている）。スタブ車両のボタン応答の問題か要確認（実機は段6a で通っている）。このため画面確認はボタンなしで通した。
- ランナーが走行モードを読む DB は `DATABASE_URL` ではなく設定の既定（本番 DB）だった（読むだけ）。
- 学習の最後まで（38 分）・チェック NG（終了コード 8）の画面は未確認（部品は終了コード表と同じ経路）。ボタン角度の保存で行末コメントの角度（例 `# 32°`）は書き換えない（古くなる）。
- 自動運転のグラフ・経過時間は走行（RUNNING）の開始からで、モード開始（準備・走行前チェックの後）からではない（今までの作りのまま）。


### 段6（2026-11-15）: タイムスケジュール・シーケンスの実行

詳細（再生の仕様・シーケンスの進め方・差し替え 5）は `src/robot/README.md`「段6」。

| ファイル | 内容 |
|---|---|
| `src/robot/schedule_spec.py`（新規） | 純粋な部分: データ・線形補間・ペダルの決め方（0〜max で頭打ち → 片足ルール → 待機位置 min）・実開度の安全網・ボタン名の検査（`check_schedule`）・押す秒数の既定 |
| `src/robot/schedule_job.py`（新規） | 開ループ再生。core の `Step`（手順番号 30・走行前チェックつき）として `run_steps` に載せる。50 ms 周期・core の `AxisSafetyNet`・過電流・最高速・CAN/通信失敗・`press_button`。中断（SIGINT）は既存の StopController で shutdown まで進む |
| `src/robot/run.py` | `--schedule ID｜名前`（DB。`DATABASE_URL` 優先）・`--schedule-file JSON`（Web が使う）。手順 3 を動かせる手順から外した |
| `src/robot/streaming_log.py` / `patches.py` | `SCHEDULE_DRIVE` を drive の CSV に。差し替え 5（スタブのボタン→車両状態の同期） |
| `src/app/job_manager.py` | 種類 `schedule`・`start_schedule`・`start_sequence`（段を `expand_runs` で 1 回ずつに展開し 1 ランナー起動＝1 セッションで順に。失敗・停止・非常停止で残りを止める）・進み具合 `sequence`/`last_sequence`・job への `sequence` |
| `src/web/routers/drive.py` ほか | `POST /drive/schedule/start|stop`（501 を実装）・`/sequence/start|stop`。始める前に全段を検査して 422（日本語の理由）。`status`・WebSocket に `sequence` |
| モデル・保存・API | `ButtonEvent.button`（`start/P/N/D`。channel・press_duration_s は任意）。DB の `button_events` は JSONB なのでスキーマ変更なし。旧データ（channel だけ）は `button=None`＝要再設定で残し、保存し直しまで channel を保つ |
| 画面 | スケジュール編集の列を ch → ボタン名（スタート・P・N・D）に。旧イベントは赤い「chN（要再設定）」列＋見出しのボタン選択で付け替え・名前なしが残ると保存不可。シーケンス一覧に「選択」。自動運転でスケジュール・シーケンスの開始を有効化し、シーケンスは「全体 a/b 周目・n/N 段目（…）・通算 k/K 回」を表示、終わると結果の帯 |

**追加指示（ユーザー決定 2026-11-15）の反映**: モード走行は常に手順 24。`buttons.enabled`・手順 3 への切り替えを、車両 yaml の雛形・
`vehicle_store`・`config_store`・API・プロファイル画面・ホーム・自動運転から外した（旧 yaml に残っていれば無視）。ランナーは手順 3 を受け付けない。
スケジュールのボタンイベントは常に有効。スタブで手順 24 が「D に入っていない可能性」で止まる件は、core のスタブ車両が
ペダルの値を車速の読み取り時にしか受け取らず、モードの 0 秒の Ready 押下（有効化の直後・車速を読む前）でブレーキ 0 と見なされて
IGN-ON(RLY-ON) になるのが原因。`patches.install_stub_button_sync`（スタブのときだけ、押す直前に車速を 1 回読む。実機は元の関数のまま）で解決。

**確認（スタブ・ポート 8081・本番 DB の複製 `driving_robot_stage6`・車両は scratchpad の複製。終了後に停止・DB 削除・複製削除）**

| 項目 | 結果 |
|---|---|
| ruff check src scripts tests/unit | 通過 |
| 足した単体テスト | `test_schedule_spec.py` 11・`test_schedule_job.py` 9（補間・頭打ち・片足・ボタン時刻と既定秒数・ボタン失敗で走行継続・CAN 失敗/最高速で DriveError と離す・cancel でボタンタスクも止まる・読み込み）・`test_sequence_run.py` 9（展開順・1 セッションずつ・失敗で残りを止める・停止（段の最中/切れ目）・非常停止・開始失敗・単発）・`test_stub_button_sync.py` 3（スタブで Ready になる・元の関数では外れる（原因の再現）・実機は元の関数のみ）。計 32 件通過 |
| 画面からスケジュール（0〜30 s・ペダル 6 点・ボタンなし）を作成 → 実行 | 作成 → 選択 → 自動運転で開始 → 終了コード 0・`drive_sessions` completed（run_type auto・mode_id なし）・`drive_logs` 661 行（section SCHEDULE_DRIVE・ref すべて NULL）・進行のメッセージ 111 行。CSV は取り込み後に消えた |
| 中止 | 再生中に `/schedule/stop` → 終了コード 130・「終了処理: 原点復帰 → サーボOFF → 切断 完了」・セッション error・ログ 283 行は取り込み済み |
| 旧データのスケジュール（本番の `Maintenance_Mode`・ch だけ 4 件） | 開始 → 422「ボタン名のないイベントが 4 件あります（1s・7s・10s・13s…）。スケジュール画面で…保存し直してください」。画面は赤い要再設定の列を出し、保存は「ボタン名が未設定です」で拒否（`.playwright-mcp/s6_02〜03`）。シーケンスに含めても 422（「2 段目: …」） |
| シーケンス（ボタンつき短いスケジュール ×2 回 × 全体 2 周＝4 回） | 4 回とも completed・セッション 4 つ・進み具合が 1/4〜4/4 で進む（status・job・ws） |
| シーケンスの途中の非常停止 | 3 回目の最中に `/emergency` → 3 回目は emergency・残り 1 回は走らず state emergency「1 段目…（1/2 回目、全体 2/2 周目）の途中で非常停止しました。残りは走らせません」。解除後は待機に戻る |
| シーケンスの mode 段（手順 24・`limit_s` 30〜60）＋ schedule 段（画面から） | schedule 段・mode 段とも completed（終了コード 0）・結果の帯 |
| 自動運転 US06（手順 24・`dev_limit_s=60`）を画面から | 終了コード 0・completed・手順 `1,24`（差し替え 5 の前は 3 秒で終了コード 5） |
| スクリーンショット | `.playwright-mcp/s6_01〜s6_16` |

**迷って決めたこと**
- Web は検査済みのスケジュールを JSON（車のフォルダの `.work/`・ジョブの終わりに削除）でランナーへ渡す（`--schedule-file`）。`--schedule ID` は CLI 用。DB なしの環境でも動かし、検査と実行の内容を一致させるため。
- 旧データの保存: API は button・channel どちらかがあれば保存可（旧テストの互換）。画面は名前なしの列が残ると保存不可。
- 終端は両ペダルを待機位置にして終える（減速して止めない）。開ループなので終端で車が動いていれば進行のメッセージで知らせるだけ。必要なら終端の緩減速（`decelerate_to_stop`）を足す案あり（実機の判断）。
- シーケンスの停止は「走っている段を止め、残りは走らせない」（段の切れ目の停止は次の段を始めない）。

**残った問題**
- **既存の src 用単体テストのうち 29 件が失敗する**（`test_job_manager.py` 13・`test_robot_runner.py` 10・`test_vehicle_servo_map.py` 2・`test_vehicle_store.py` 2・`test_robot_config_store.py` 2）。`buttons.enabled`・`mode_drive_steps(bool)`・`check_job(buttons_enabled=…)`・`JobManager(buttons_enabled=…)`・`store.create(buttons_enabled=…)` を使うため。制約「tests/ の既存ファイルは変更しない」に従って**触っていない**。直すなら該当引数・期待値を外すだけ（許可が要る）。
- 実機の経路（`--hw real` のスケジュール再生・シーケンス・ボタン押下・非常停止の所要秒数）はスタブでは動かしていない（段8）。
- スケジュール画面の編集で、押す時間が無い（既定秒数の）イベントは表示上 1 秒として扱うので、保存し直すと 1 秒が明示される。

### 段7（2026-11-15）: 古いコードの削除・テストの修正・スケジュール終端の減速・文書

**決定の反映**: A-3（tests/research だけ触らない。tests/unit・integration・hardware の旧テストは古いコードと一緒に削除・新しい src のテストは直す）・D-15（モード走行は常に手順 24）。
`src/robot/core/` と `tests/research` は変更していない（`git status` に差分なし）。

**1. 段6 で失敗していた 29 件を直した**（テストの意図は保ち、今の API に合わせた）

| ファイル | 直したこと |
|---|---|
| `test_job_manager.py`（13） | `JobManager(buttons_enabled=…)` を外す。`mode_drive_steps(bool)` のテストは「`MODE_DRIVE_STEPS` は常に `1,24`」に。「ボタンなしの車は手順 3」の確認は「モード走行は 1,24」に |
| `test_robot_runner.py`（10） | `check_job(buttons_enabled=…)` を外し、手順 3 は「動かせません」・`--mode` の文言は「手順 24」に合わせた |
| `test_vehicle_servo_map.py`（2）・`test_vehicle_store.py`（2）・`test_robot_config_store.py`（2） | `buttons_enabled=`・`buttons.enabled`・`config_store.buttons_enabled` の使用を外し、旧 yaml に `enabled` が残っても合成に入らないことの確認に |

そのほか: `test_web_sequences.py`・`infra/test_schedule_repository.py` は消した `src.app.stubs` ではなく `src.app.inmemory_repos` を使うよう直した。
`test_ws_broadcast.py` は旧 `RobotController` 前提の 6 件を、JobManager の状態を配信することを見る 2 件（非常停止の状態が配信される・不変なら再送しない／新しい接続には送る）に置き換え（ストールした接続の隔離の 1 件はそのまま）。
`test_models.py` からは削除したモデル（プロファイル・キャリブレーション）の部分だけ外した。

**2. スケジュール再生の終端（Claude 決定・安全側）**: 段6 は終端で両ペダルを待機位置にして終わり、車が走ったまま終了処理に進んでいた。
core のモード走行の終わり（`mode_drive._finish_stopped`）と同じにした（`src/robot/schedule_job.py` の `_finish_stopped`）:
車速が停止判定（`VEHICLE_STOP_SPEED_KMH`）以上なら core の `stop_decel.decelerate_to_stop`（モード走行の DECEL_TO_STOP と同じ関数）で停車まで減速して停車保持、
止まっていればアクセル原点・ブレーキ停車保持開度にしてすぐ返す。その後に終了処理。区間名 `DECEL_TO_STOP`（APPROACH → STEP → STOP_HOLD）。
中断（SIGINT）は減速中でも手順本体の cancel → 終了処理へ進む（core のまま）。

| 確認 | 結果 |
|---|---|
| 単体（スタブ・本物の `decelerate_to_stop`、刻みの待ちだけ短縮） | 30% 相当で走った車が停止し、アクセル 0・ブレーキが停車保持開度以上 |
| 単体（すでに停車） | 減速は呼ばず `STOP_HOLD` を記録して停車保持 |
| 画面から実行（スタブ・0→8 s で 30%・14 s まで保持・15 s で 0%、最高 55.0 km/h で終端） | 終端の車速 54.79 km/h → 「緩減速で停車させます」→ **26.1 秒で停車**（最大減速 0.126G・踏み増し 25 回・戻し 0 回）→ 停車保持ブレーキ 18.09% → 終了処理 完了。ジョブ全体 44.3 秒・終了コード 0・completed |

スタブ車両は実車と別物（応答が違う）ので、秒数は実機の目安にならない（実機の減速は段8 で見る）。減速の目標は core の `decel_stop.target_decel_g`（0.2G）のまま。

**3. 古いコードの削除（B-5）**: 消す前に、`src.web.app`・`src.robot.run`・`src.robot.interactive` から辿れる import（`ast` で数えた）と `grep`（src・scripts・tests）で参照を確かめ、
Web・ランナーから使われないもの（と、それだけを試すテスト）を消した。計 **80 ファイル**（`src/domain` の `__init__` 2 本を含む）。

*src（37 ファイル）*

| ファイル | 理由 |
|---|---|
| `src/app/robot_controller.py` | 旧制御の本体。Web は JobManager + ランナーに置き換え済み |
| `src/app/learning_cycle.py`・`plan_service.py`・`training_service.py` | 旧学習サイクル・プラン・モデル訓練（研究の手順 2 に置き換え） |
| `src/app/stubs.py` | 旧スタブ（旧 RobotController 用）。in-memory リポジトリは `inmemory_repos.py` が引き継いでいる |
| `src/app/factory.py` | 旧 RobotController の組み立て |
| `src/domain/control/*`（drive_loop・learning_loop・pedal_plan・plan_update・trim・reward・pid・feedforward・kpi_monitor・pedal_arbiter・base_loop・schedule_loop・conversions・pedal_safety と `__init__`。15 本） | 旧制御。core の写しに置き換え。`schedule_loop` の考え方は `schedule_job.py` に移した |
| `src/domain/calibration.py`・`learning_drive.py`・`model_training.py`・`pid_tuning.py`・`pre_check.py`・`safety_monitor.py` と `__init__`（7 本） | 旧キャリブレーション（min 測定に置き換え）・学習運転・PID 適合・走行前チェック（core のもの）・安全監視（JobManager の非常停止） |
| `src/infra/actuator_driver.py` | 旧アクチュエータドライバ。実機は core のドライバ。使うのは消す旧テストと旧 tests/hardware だけだった |
| `src/infra/button_servo_driver.py` | 旧ボタンサーボ。core の `button_steps`/`hardware` が持つ |
| `src/infra/profile_repository.py`・`pedal_plan_repository.py` | プロファイル（DB）・プラン（DB）は使わない（C-10・B-5）。DB の表はデータを残す |
| `src/infra/archive_manager.py` | どこからも使われていなかった（PRD のアーカイブ項目は未実装のまま。必要なら作り直す） |
| `src/models/calibration.py`・`profile.py`・`pre_check.py`・`learning_drive.py` | 上の旧コードだけが使うモデル |

*scripts（8 本）*: 旧プロファイル・旧制御を前提にした分析・模擬の道具 — `analyze_decel_fit.py`・`analyze_session.py`・`compare_theta_id.py`・`evaluate_feature_sets.py`・`preview_pedal_plan.py`・`replay_plan_update.py`・`restore_coast_curve.py`・`simulate_control.py`（`src.domain`・`profile_repository`・`models.profile` を import し、消した後は動かない。研究側の分析は `research-run-analysis` スキル・tests/research にある）。

*tests（35 ファイル）*: tests/research は触っていない。
- `tests/unit`（27）: `test_calibration`・`conversions`・`drive_loop`・`factory`・`feedforward`・`kpi_monitor`・`learning_cycle`・`learning_drive`・`learning_loop`・`model_training`・`pedal_arbiter`・`pedal_plan`・`pedal_safety`・`pid`・`pid_tuning`・`plan_service`・`plan_update`・`pre_check`・`review_fixes`・`reward`・`robot_controller`・`safety_monitor`・`schedule_loop`・`stub_calibration`・`training_service`・`trim`・`web_drive`（すべて消した旧コードを試すもの。`test_web_drive` は旧 RobotController の API で 50 件失敗していた）
- `tests/unit/infra`（5）: `test_actuator_driver`・`archive_manager`・`button_servo_driver`・`pedal_plan_repository`・`profile_repository`
- `tests/integration`（1）: `test_web_api.py`（旧 RobotController・旧モデルの API テスト）
- `tests/hardware`（2）: `test_emergency_stop_home_return.py`・`test_overcurrent_home_return.py`（消した旧 `ActuatorDriver` を使う実機確認。非常停止の原点復帰は JobManager→ランナー SIGINT の経路になった。bench-verification・memo_codeRun に代わりの確認を書いた）

**残したものと理由**

| 残したもの | 理由 |
|---|---|
| `src/infra/can_reader.py`・`scripts/check_can.py`・`check_can_2.py`・`check_can_3.py`・`record_can_speed.py`・`tests/hardware/test_can_receive.py`・`tests/unit/infra/test_can_reader.py` | CAN の読み取り確認の道具（実機の配線確認に今も使う。memo_codeRun に手順あり）。制御には使わない |
| `src/infra/gpio_monitor.py`・`ups_monitor.py`・`log_buffer.py`・`log_writer.py` | Web 側が使っている（非常停止・UPS・サーバーログ） |
| `src/infra/settings.py` の `ArchiveSettings`・`config/settings.toml` の `[archive]` | 設定の読み込みが壊れないよう残した（アーカイブ機能は未実装） |
| `tests/hardware/test_calibration.py` | `tests.research.actuator_driver` を使う min 測定の道具で src を import していない。そのまま使える（WebUI のキャリブレーション画面でも測れる） |
| `tests/hardware/test_emergency_stop.py`・`test_servo_pca9685.py`・`test_ups_monitor.py` | GPIO・サーボ・UPS の実機確認（古い src を使っていない） |
| `scripts/setup_db.py` | DB の旧テーブル（`vehicle_profiles`・`learning_cycles`・`pedal_plans` など）の作成文も含めて残す（データを残すだけで使わない） |
| `scripts/port_research.py`・`import_research_vehicle.py`・`check_config_equivalence.py`・`check_runner_equivalence.py`・`check_runner_interrupt.py`・`setup_nut.sh`・`setup_udev.sh` | 移植道具・等価確認・環境設定。`check_runner_interrupt.py` の既定手順を 1,3 → 1,24 に直した |

**4. 文書・5. requirements.txt**
- `docs/product-requirements.md`: プロファイル・キャリブレーション・学習運転・自動運転・タイムスケジュール・ログ管理に「2026-11-15 の変更」を足した（北極星なので項目の削除はせず、置き場所・制御の中身・未実装の印だけ）。
- `docs/memo_codeRun.md`: ランナーの実行例・車両フォルダ・移植道具の使い方・取り込み直しの手順・環境変数を追記。削除した hardware テストの記述を置き換えた。
- `docs/hardware.md`: 制御ソフトとの対応（シリアルを握るのはランナーだけ・ボタンサーボは車両 yaml・非常停止の流れ）を追記。
- `.claude/skills/bench-verification/SKILL.md`: 今の作り（JobManager→ランナー・`--hw real`・非常停止は SIGINT→40 秒で SIGKILL）・車両フォルダ・DB の複製・走行系のスタブ確認の手順・禁止事項の更新。
- `src/robot/README.md`: 終端の減速・手順 3 の記述の除去・段7 の節を追記。
- `requirements.txt`: 消した後の import（`src` `scripts` `tests/unit` `tests/integration` `tests/hardware`）を数え直して見直したが、載せているライブラリはどれも core・Web・残したモジュールが使っているので変更なし（`pymodbus`・`python-can`・`cantools`・`smbus2`・`matplotlib` は core、`asyncpg`・`fastapi` は Web）。

**確認の結果**

| 項目 | 結果 |
|---|---|
| `ruff check src scripts tests/unit tests/integration tests/hardware` | 通過 |
| `pytest -q tests/unit` | 全件通過（失敗 0） |
| `pytest -q tests/integration`（ローカル PostgreSQL） | 9 件通過 |
| `python -c "import src.web.app"` | OK（`src.app.job_manager`・`src.robot.run` も） |
| `tests/research` の変更 | なし（`git status` で確認。`src/robot/core` も手で直していない） |
| 画面（JS）が外した API（`/plan`・`/pid-tune`・`/pause`・`/resume`・`set-zero`・`set-full`・`learning/train`）を呼んでいないか | `grep` で該当なし |
| スタブで uvicorn（ポート 8081・本番 DB の複製・車両は scratchpad の複製。終了後に停止・DB 削除・複製削除） | 初期化（手順 1）終了コード 0・モード走行 `03_WLTP_Low`（手順 1,24・`limit_s` 60）終了コード 0・スケジュール（上の表）終了コード 0・終端で 26.1 秒で停車 |

**残った問題**
- 実機（`--hw real`）の経路（スケジュール終端の減速・ボタン押下・非常停止の所要秒数）はスタブでは動かしていない（段8）。
- `bench-verification` スキルは「Claude が `DRIVING_ROBOT_USE_REAL_HW=1` で WebUI を起動するのは禁止」のままにした。段8 の計画（GUI からのモード走行を Claude が無人で）とずれるので、段8 の前に決めが要る。今は「ランナーの直接実行（`python -m src.robot.run --hw real`）は tests.research と同じ扱いで Claude が実行してよい」と書いた。
- 旧 `.claude/worktrees/*` の中身（古いコピー）は触っていない。

### 段7b（2026-11-15）: 学習運転のボタン操作（D-14b）

**決定の反映**: D-14b（src の学習運転＝手順 2・12〜14 も、手順 24 と同じく Ready・D を押してから走り、終わりに N）。tests/research と `src/robot/core` は変えていない。

**変えたもの**
- `src/robot/learning_buttons.py`（新規）: 学習の手順の `Step.handler` を包む。`run.py` が `wrap_step` の後に `add_button_ops` で足す。差し替え点の表に #6 として記載（`src/robot/README.md`）。
- `src/robot/run.py`: `add_button_ops` の呼び出し・`check_job` に「学習の手順と手順 23・24 は同じ実行にできない」を追加。
- `src/robot/events.py`: state の phase に `ready`・`d`・`n`。
- `src/web/static/js/job-utils.js`・`screens/learning.js`: 段階表示に「Ready・D」（準備の次）と最後の「N」を追加・説明文に「Ready・D →」「→ N」・PHASE_LABEL。
- `tests/unit/test_learning_buttons.py`（新規）。

**押す順序・待ち時間**（手順 24 の関数を使う）
| 順 | 操作 | 待ち |
|---|---|---|
| 1 | 走行前チェック（run_steps） | |
| 2 | ブレーキを停車保持の開度に（`set_brake(on=True)`。値は進行のメッセージに出す） | |
| 3 | スタート短押し（Ready） | 押す間 `start_short_press_s` |
| 4 | Ready の押し始めから `buttons.d_press_at_s` 経つまで待つ（車速を読み続ける）→ D | D を押す `shift_press_s` |
| 5 | D の後 `after_press_wait_s` 待つ → 学習の handler | |
| 6 | 最後の学習の手順の後（エラーでも）`press_shift_n_after_drive`（ブレーキ ON で N） | N の後 `after_press_wait_s` |

`1,13,14` のように続くときは Ready・D・N とも 1 回。SIGINT/SIGTERM の中断では N を押さず shutdown。

**確認の結果**（スタブのみ。実機・ポート 8080・シリアルには触れていない。リポジトリは書き換えず、複製で確認）
| 項目 | 結果 |
|---|---|
| 単体（順序・回数・中断で N なし・エラーで N・N 失敗時の扱い・state イベント・core の関数の順序と待ち時間・check_job） | 通過 |
| `ruff check` | 通過 |
| スタブ `--steps 1,13` | pre_check → Ready → D → 手順 13 の走行開始まで確認。SIGTERM で中断 → N は押さず shutdown（終了コード 143） |
| スタブ `--steps 1,12` | phase が ready → d → step の順で走行開始 |
| スタブ（手順 13 の handler を擬似エラーにした実行） | Ready → D → エラー → N → shutdown（終了コード 5）。state は ready, d, step, step_end, n, shutdown |

**迷って決めたこと**
- 段階表示の「Ready・D」は、時間順に合わせて「準備（初期化・走行前チェック）」の次に置いた（依頼文の「先頭」は準備の前とも読めるが、走行前チェックの後に押すため）。
- 学習の手順と手順 23・24 の同居は設定エラー（Ready のスタートを 2 回押すと IGN-OFF になるため）。
- 学習の手順が Ready の途中で失敗したときも N を押す（状態が分からないので戻す側）。
- N の失敗は、学習が成功していればエラー（終了コード 5）、学習がすでに失敗していれば元のエラーを優先。

**残った問題**
- 実機での確認は未実施（実機は別作業が使用中）。停車保持の開度が仮値のままの新しい車では、その値で踏む。

**D-15b の反映（同日・通しの流れ）**
- モード走行は `1,21,22,24,25`、学習運転は `1,21,22,<2|12|13|14>,25`（JobManager の `MODE_DRIVE_STEPS`・`LEARNING_STEPS`。シーケンスのモードの段も同じ）。Ready・D・N は学習の手順の前後にランナーが足す（上）ので、学習は 21 → 22 → 走行前チェック → Ready・D → 学習 → N → 25 の順。
- 学習の手順の準備（読み込み）は core の `preload_target` により 21・22 の前に済む（スタブで確認: 準備 → 21 → 22 → 走行前チェック）。`check_job` は学習と 23・24 の同居を拒否し、21・22・25 は許す。
- 中断（SIGINT）・非常停止では 25 も N も押さない。エラーで終わったときは core の `run_steps` が後の手順へ進まないので 25 は押さない（学習は N だけ押す）。車は Ready のまま残る。
- そのため `JobStatus.needs_ign_off`（手順 21・22・24 または Ready・D を始めたのに 25 が正常に終わっていない学習・走行ジョブ）を `last_job` に付け、画面の結果の帯（自動運転・学習運転）と非常停止の画面に「車は Ready のまま。次の走行の前に IGN-OFF にしてください（手順25 のボタン）」と「IGN-OFF にする」ボタンを出す。ボタンは新しい API `POST /api/v1/drive/ign-off`（ジョブ種別 `ign_off`・手順 `1,25`）。
- 画面: 学習の段階表示を「初期化・IGN-ON・N ホールド・走行前チェック・Ready・D・（学習）・N・IGN-OFF」に、説明文・自動運転の説明も通しの流れに更新。

**D-15b の確認の結果**（スタブのみ）
| 項目 | 結果 |
|---|---|
| `ruff check src scripts tests/unit`・`pytest tests/unit` | 通過・322 件通過 |
| ランナー `1,21,22,24,25 --limit-s 60` | 終了コード 0（準備 → 21 → 22 → 走行前チェック → 24（Ready 0 秒・D 3 秒・N）→ 25） |
| ランナー `1,21,22,13,25` を Ready・D の後に SIGINT | 終了コード 130・N も 25 も押さず shutdown |
| uvicorn 8081（本番 DB の複製）: 画面 API の `drive/start`（limit_s 60） | steps が `1,21,22,24,25` で終了コード 0・`needs_ign_off` false |
| 同: 学習 `hold` を開始 → 画面の段階表示 → `drive/stop` | 段階が初期化 ✓・IGN-ON ✓・N ホールド ✓・走行前チェック ✓・Ready・D ✓・ペダル探索 ▶。停止後に `needs_ign_off` true で「車は Ready のまま」の帯とボタンが出た |
| 同: 「IGN-OFF にする」を押す | `ign_off` ジョブ（1,25）が終了コード 0・`needs_ign_off` false |
| 後始末 | uvicorn 停止・DB の複製・車両の複製を削除 |

**D-15b で迷って決めたこと**: `needs_ign_off` は「手順 21・22・24 か Ready・D を始めたジョブが 25 を正常に終えなかった」ときに付ける（手順 1 や設定エラーで止まった場合は車が動いていないので付けない）。IGN-OFF ジョブは画面の履歴（last_job）を置き換えるので、押すと帯は消える。

### 段8（2026-11-15）: 実機確認（途中）

**GUI からのモード走行（WLTP、手順 1,21,22,24,25）が 2 回とも D で走り出さない**
| 時刻（JST） | 走行 | 結果 |
|---|---|---|
| 12:14〜12:17 | GUI WLTP（セッション 07d9e9d4…） | 終了コード 5「基準車速が出てから 3s たっても 0.2 km/h 以下（D に入っていない可能性）」 |
| 14:20〜14:24 | GUI WLTP（セッション d2578b0c…）。直前に「D でブレーキを離しても 0.00 km/h」で IGN-OFF を確かめてから | 同じ終了コード 5。ボタンの順・角度・時刻は成功した tests の US06（120031）と同じ |

**調べたこと（scratchpad の診断。ボタンの押し方は core の関数のまま）**
- シミュレータ（`~/projects/driving_simulator/src/vehicle_status.py`）の状態遷移では、止まっていればボタンの押し方だけで状態が決まる。キーリピートは無視するので二重押しにはならない。
- ブレーキの待機位置 4.20% でのシミュレータの読み値（CAN 0x121）は 0.000%。停車保持 18.09% は 27.3%。ペダルの読み値には問題がない。
- 「リセット（ブレーキを離してスタート 6 s 長押し → 止まっていればどの状態からでも IGN-OFF）→ ブレーキ ON → スタート短押し → D → ブレーキを離す」を繰り返すと:
  - 1 回目のプロセス: 動いた・動いた・動かない × 3
  - 2 回目（スタート 32° と 30° を交互）: 動いた・動いた・動かない × 6（角度は関係なし）
  - 3 回目: 4 回とも動かない（押す直前のブレーキ読み値は 27.3%・離した後は 0.0%）
- 結論（推定）: ボタンの押下（スタートの短押しか D。どちらかは車速だけでは区別できない）が、繰り返すうちに効かなくなっている。src のコードの違いではなく、ボタンのサーボ・キー側の問題とみられる。実物の確認（サーボの角度・ホーンの緩み・発熱・キーの位置、シミュレータの画面の State/Range）が要るので、ユーザーの確認待ち。
- 診断の終わりはリセットの長押しで IGN-OFF のはず（押下が効いていれば）。uvicorn は停止・シリアルは空き。
- 追加の切り分け（同日）:
  - 切り分け試験（リセット → スタート → D。動かなければ D をもう一度 → スタートをもう一度 → D）: 動いた・動いた・3 段とも動かない × 2。
  - Ready に 1 回入れて D だけを 8 回（間に 60 s 休み）: 8 回とも動かない。
  - 5 分休ませてから切り分け試験: 動いた・3 段とも動かない × 2。
  - 休ませると最初の 1〜2 回は効き、数回押すと効かなくなる（60 s の休みでは戻らず、5 分以上の休みで戻る）。車速しか見えないので、スタートと D（N と同じ ch1）のどちらが効かないかは決められない。サーボの電源・発熱・キーの機構を実物で見る必要がある。
  - 再開（同日 17:2x〜17:49）: ユーザーが戻った後、休ませた状態から 手順 1,23（Ready）→ scratchpad の一回きり（ブレーキ ON → D → 離す）で**クリープが出た**（0.96 → 5.00 km/h、14 s で一定）。押したのはスタート 1 回・D 1 回・N 1 回だけ。
    - その直後（離してから 19 s）に車速が 5 → 15 km/h（安全策でブレーキ）→ N の 3 s 後 97.28 km/h、その後 89 km/h から約 0.75 km/h/s で惰行。アクセルは待機位置のまま。原因は未確認（シミュレータの読み値の乱れと推定）。
    - 続けて GUI（API）から WLTP（手順 1,21,22,24,25、セッション 76bcad16…、CSV `drive_log_real_20261115_174853.csv`）: **同じ終了コード 5**。手順 21・22 で 30 s のうちに 6 回押した（スタート長押し・P・N・P・N・N）後の Ready（スタート）・D が効かなかった。休ませると 1〜2 回は効き、数回押すと効かなくなる、という前回までの見立てと同じ形。ユーザーの実物確認待ち（UDDS・US06・② 手順2・③ 非常停止は未実行）。

#### ① GUI（API）からのモード走行（同日 17:58〜19:09、手順 1,24,25・IGN-OFF はスタート 10 s）
3 本とも終了コード 0・completed・needs_ign_off false（ボタンの効かなさは出なかった）。DB からログ画面と同じ `/sessions/{id}/logs.csv` で書き出した CSV は tests の CSV と同じ 83 列（見出し一致）で、research-run-analysis の道具がそのまま使えた。書き出しは WLTP 36087 行で 2.5 s。

| 走行 | CSV | 最大 | p95 | \|偏差\|>0.5 [s] | >1 [s] | 往復 全体 | 往復 60s 最大 |
|---|---|---|---|---|---|---|---|
| WLTP tests 本体 | real_20261115_064023 | 0.56 | 0.22 | 0.85 | 0.00 | 0.89 | 1.46 |
| WLTP GUI | real_20261115_180059（セッション 949d500d…） | 0.62 | 0.22 | 1.05 | 0.00 | 1.01 | 1.71 |
| UDDS tests 本体 | real_20261115_095244 | 1.26 | 0.25 | 6.25 | 1.00 | 1.06 | 1.53 |
| UDDS GUI | real_20261115_183359（3e72a59c…） | 1.00 | 0.27 | 6.20 | 0.00 | 1.37 | 2.00 |
| US06 tests 本体 | real_20261115_120031 | 0.74 | 0.34 | 5.75 | 0.00 | 1.46 | 3.32 |
| US06 GUI | real_20261115_185831（a33c8ac4…） | 0.68 | 0.33 | 3.15 | 0.00 | 1.44 | 3.21 |

- 設定は tests の走行 yaml と同じ（違いは置き場所・plot・モード名・走行抵抗の読み方だけ）。制御周期も同じ（中央 0.050 s・p99 0.071 s。最大は GUI 0.085 s／tests 0.23 s）。
- 偏差の KPI は同等。往復は W +0.12・U +0.31 増えたが、UDDS で増えた窓は全体に散らばり、惰行の補助・遊びの下限・保持の印はほぼ 0 s。等価確認② でペダル指令は一致しているので、GUI 経由のせいではない（推測: 日・時間帯のばらつき）。

#### ③ 画面の非常停止（同日 19:12〜19:16、写しの車フォルダ、US06 の手順 24 開始から 60 s・約 70 km/h で `/drive/emergency`）
- `/emergency` → ランナーの「終了処理: 完了」まで約 5 s、ジョブ終了（DB 取り込み込み）まで 8.0 s。終了コード 130・セッション emergency・1215 行取り込み。`KILL_TIMEOUT_S` 40 s には十分余裕。
- SIGINT の瞬間に読み取り中の Modbus が取り消され「アクチュエータ通信に失敗しました（Request cancelled outside library）」→「異常終了: ペダルを離します」の文が出る（原点復帰は済む。表示だけの問題）。
- 後始末（scratchpad の一回きり: ブレーキ ON → 5.2 s で停車 → N → スタート 10 s）は正常。
- **`/reset-emergency` が 423**（「非常停止スイッチがまだ押されています」）。GPIO17 を読むだけで確かめると HIGH（NC 接点の「押下／開放」）が続いている。物理スイッチが押されているか、配線が外れている。開いたままだと押してもエッジが出ず物理の非常停止が効かない。また `/start` は GPIO のレベルを見ずに走り出せる（今日の走行はこの状態で走った可能性）。ユーザー確認待ち。② 手順2 は未実行。

### 段9（2026-11-16）: ボタンなし・初期化ページだけで初期化（D-15c）
計画: ① ボタンなしの設定・GPIO を切る設定・手順の並び（モード 24,25・学習 <n>,25）・初期化済みでないと開始できない、を実装（sonnet）。② スタブで tests と src の手順2 の指令を突き合わせる。③ 実機（シミュレータは常に Ready・D）で画面を順に確認しスクリーンショットを送る: 初期化 → 車両プロファイル → キャリブレーション（min を 5.0 にしてから画面のジョグで 4.2 へ戻して保存・max はプロファイルで 61.0・47.3）→ 自動運転 WLTP（同じ日の tests `--steps 1,3` と比べる）→ 学習運転 手順2（車フォルダを写して残す・tests の yaml の md5 が変わらない）→ ログ。

#### 実装結果（sonnet、2026-11-16）
- **手順**: モード `24,25`・学習 `2,25`/`12,25`/`13,25`/`14,25`・IGN-OFF `25`・初期化ページ `1`。手順 1 を含まないジョブは `src/robot/quiet_connect.py`（接続・Modbus 操作権・アラームリセット（出ているときだけ）・サーボ ON。原点復帰なし・表なし・init イベントなし）を `run.py` が `run_steps` の前に呼ぶ。core `run_steps` は `ctx.hardware` が初期化済みでないと `require_hardware` で拒むので、遅延接続ではなくこの前処理にした。
- **関門**（`JobManager`）: 初期化ジョブ正常終了で true／非常停止・0 以外の終了・サーバー起動で false。学習・モード走行（シーケンスのモードの段も）は false で 409＋日本語の案内。`/drive/status`・WebSocket に `initialized`、`/drive/status` に `devices`。画面: ホームに初期化の状態、自動運転・学習の開始ボタンを無効化して「初期化ページへ」、初期化ページに「現在: 初期化済み/未」と UPS・非常停止(GPIO)・SG90 の行（off は「なし（設定で off）」）、段階表示・説明文を新しい流れに。
- **ボタンなし**: 差し替え 8（`patches.install_buttons_off`）。`buttons.installed: false` で `press_button` は押したことにしてログ（`ボタンなし: 「…」を押したことにします`・秒数は待つ）、`ensure_buttons_connected` は PCA9685 に繋がない。`vehicles/test_vehicle/vehicle.yaml` に `installed: false`。プロファイル API の読み書き対象（キーが無い古い yaml は行を足して書く）。
- **GPIO**: `[web_gpio] enabled = false`（settings.toml）・環境変数 `DRIVING_ROBOT_GPIO_DISABLED=1`。AC 断の UPS コールバックは GPIO の見張りが無くても繋ぐように直した。`python -m tests.research.main --list` が通ることを確認。
- **決めたこと**: ① 設定キーは `[gpio]` に足さず `[web_gpio]`（tests/research が `[gpio]` の未知キーを拒否するため）。② IGN-OFF ジョブ（25）・対話（手動・キャリブレーション）・スケジュール（手順 1 を自分で行う）は関門なし（エラー終了の後に車を IGN-OFF へ戻す手段が要る・キャリブレーションは設定の一部）。③ 利用者の停止（終了コード 130）も「正常でない終了」として関門を閉じる（指示どおり。不便なら 130/143 だけ除く）。④ 初期化済みの印が false でも `robot_state` は READY のまま（画面が `initialized` で判断）。
- **確認**: ruff 通過・tests/unit 339 件通過（`test_buttons_off.py`・関門のテスト・手順の並びのテストを更新/追加）。スタブの uvicorn（8081・DB なし・車フォルダ複製）: 初期化前の学習開始は 409／初期化 → `initialized` true／モード走行（WLTP・60 s）は手順 `24,25`・終了コード 0・`ボタンなし:` の行（Ready・D・N・IGN-OFF）・初期化の表なし（init_steps 空）／学習 `hold` は `13,25`・Ready の `[学習]` の行・停止（130）で `initialized` false・`needs_ign_off` true。複製の車両 yaml は元と同一で壊れていない。uvicorn と複製は削除。実機は未確認。

#### 段9 実機の画面確認（2026-11-16、シミュレータは常に Ready・D、ボタン・非常停止・UPS なし）
| ページ | 結果 |
|---|---|
| 初期化 | 手順 1・終了コード 0。UPS・非常停止(GPIO)・SG90 は「なし（設定で off）」でスキップ。`initialized` true |
| 車両プロファイル | 画面で max 60/47・min 5.0 に変えて保存 → yaml に反映。後で max 61.0/47.3 に戻して保存。min・max 4 つは tests の yaml と一致（違いはコメント前の空白だけ） |
| キャリブレーション | ジョグでブレーキ・アクセルとも 4.20%（399 pulse）→「ここを min として保存」で yaml 4.2。**直した**: ジョグの量が pulse（+0.1 ＝ 10 pulse ≒ 0.105%）なので 4.42% のような半端な値になり 4.2 に合わせられなかった → `interactive.jog_target` を 0.1% の目盛りに丸める（テスト追加） |
| 自動運転 WLTP | 手順 24,25・終了コード 0（セッション 74b3ebda…）。同じ日の tests `1,3`（`drive_log_real_20261116_083849`、1 本目は CAN 無音 0.2 s で 171 s で中断・2 本目で完走）と同等: 最大 0.57/0.56・p95 0.24/0.23・\|偏差\|>0.5 0.65/0.35 s・>1 0/0 s・往復 1.06/1.12。等価確認① 一致（443 キー）。**直した**: 進み具合がジョブ開始（計画づくりの数分を含む）からの壁時計で、モードの時刻とずれていた → sample の `mode_time_s`（学習は `t_s`）に起点を合わせる |
| 学習運転（手順 2,25） | スタブで tests と src の手順2 を並べて走らせた: 同じ区間（走行前チェック・ペダル探索・G ランプ 15 段・停車）・同じ平均開度、指令は時刻のゆらぎで徐々にずれる（行 238 で 0.5% 差）。どちらもスタブで 4 点を保てず同じ ConfigError（既知）。**実機**（セッション 6402989c…、40 分）: G ランプ 15 段まで完走し pkl `fwd_model_20261116_111047.pkl`・`fwd_table_20261116_110551.pkl` を `vehicles/test_vehicle/models/` に保存。ただし **4 点の保持が 4 点とも保てず**（帯 0.7〜1.7 s／5 s 必要）、学習結果チェックで ConfigError（`launch_brake_band` に pkl の meta の hold_pi・hold_opening が要る）→ 終了コード 2。model_path は据え置き。2-0 の保存で車両 yaml の `stop_brake_offset_pct` が 13.89 → 14.21 に書き換わった（tests も同じ動き）。tests の yaml の md5 は不変。**直した**: 進行のメッセージ欄が自動スクロールを止めて数分前の行のまま見えていた → 止めるのはユーザーが上へスクロールしたときだけ |
| ログ | 自動運転・学習運転のセッション・図・全列 CSV が出る |

残り: 4 点の保持 NG（tests の制御と同じ。2026-11-01 から残る）をどうするか・車両 yaml の stop_brake_offset_pct を戻すか（ユーザー判断）。小さな表示: UPS off でも右上に「UPS AC OK」、学習運転のログに自動運転向けの合格判定が出る、車両プロファイルの「SG90 が付いている」チェックの見た目は未確認。

### 段10（2026-11-16 計画）: 手順2 で走れたら必ずモデルを使う（4 点の保持 A＋C・チェックの分け方）
**ユーザー決定（2026-11-16）**
- 4 点の保持の NG は tests で直してから src に取り込む。シミュレータのブレーキの読みは 183 目盛り（1 目盛り ≒ ロボットの開度 0.22%）で、1 目盛りで加速度が約 3 km/h/s 跳び、帯（±0.005 G）に 5 s いるのは当たり外れ（10-30 4/4・11-10 1/4・3/4・11-16 0/4）。
- **A**: 点ごとに、保持中の加速度の符号が変わる瞬間（+→− と −→+。停車中 0.3 km/h 未満は除く）の開度（ブレーキの遅れ L だけ前）を集め、平均をその点の保持の開度、瞬間の車速の平均をその点の車速にする。**瞬間が 2 回以上**、または今の「帯に合計 5 s」のどちらかで「保てた」。保てた点があれば hold_pi も今の求め方で求める。机上（11-10 の 2 本・11-16）で 4 点とも求まり、点の中のばらつき ±0.01〜0.19%。
- **C**: それでも求まらないときは前の pkl の hold_pi・hold_opening を引き継ぎ、進行のメッセージに「引き継いだ」とだけ残す（学習運転ページ・ログページのメッセージ。画面の帯には出さない）。前が無ければ発進のブレーキの帯を使わない設定で終える。
- **チェックの分け方**: 走り終えたら原則モデルを使う（yaml を書く）。止める（前のモデルのまま）のは C2（書いた yaml の pkl を読めない）・D1 の NaN・B1（学習データでの外れ。乱数を替えた学習し直しの後も外れる）だけ。ほかの NG（B3・B4・B5・B6・C1・D1 の範囲外・穴埋め・先置き超過・H1）は「注意」としてチェック表に残し、モデルは使う。前の pkl は消さない。

**段**（tests → 取り込み → 実機）
| 段 | 内容 |
|---|---|
| 10-1 | tests/research: A（4 点の保持の判定と値）・C（引き継ぎ）・チェックの「止める／注意」の分け方。単体テスト。机上: 11-10 の 2 本・11-16 の CSV で A の値が上の机上と一致 |
| 10-2 | 実機（tests）: 手順13（4 点の保持だけ）で A が通ること |
| 10-3 | `scripts/port_research.py` で src へ取り込み直し → 等価確認①② |
| 10-4 | 実機（src・画面）: 学習運転（手順2）→ yaml が新しいモデルを指す・注意の表示・ログ。続けて自動運転 WLTP を新しいモデルで 1 本 |

#### 10-1 結果（2026-11-16。tests/research のみ・src は未変更）
- **A**: `tests/research/hold_crossing.py`（純関数。CSV・オンライン共通）。加速度 ＝ 車速の中心差分（幅 0.4 s）・点の始まりから 3 s は除く・車速 0.3 km/h 未満の瞬間は除く・開度 ＝ t − L のブレーキ実開度（L ＝ pkl meta brake_lag_s）。`pedal_search.apply_crossings` が点ごとに「帯 5 s」または「瞬間 2 回以上」で保てたとし、瞬間で保てた点は車速・開度を瞬間の平均にする（帯で保てた点は今の値のまま）。点のデータに `criterion`（band／crossings）・`crossings` を記録。`attach_hold_data`・手順13 が使う。机上 CLI: `python -m tests.research.hold_crossing CSV --lag 0.155`。
- **机上（L ＝ 0.155 s、瞬間の開度の単純平均）**

| CSV | 点 1 | 点 2 | 点 3 | 点 4 | 瞬間の数 |
|---|---|---|---|---|---|
| 20261110_085539 | 7.98 | 7.89 | 7.87 | 7.84 | 6,6,6,4 |
| 20261110_181106 | 7.78 | 7.82 | 7.77 | 7.55 | 2,2,2,6 |
| 20261116_103054 | 8.47 | 8.43 | 8.34 | 8.03 | 3,3,5,4 |

- **C**: 保てた点が無い・hold_pi を求められないときは、前の pkl（手順2 の前の yaml の model_path）の hold_pi・hold_opening を引き継ぎ「引き継いだ」を 1 行出す（チェック H1 は注意）。前も無ければ付けず、`launch_band_for`・`make_hold_scene` は警告を出して使わない扱い（ConfigError にしない・yaml キーは書かない）。手順13 は hold_pi が無くても 1 刻み寄せで走る。
- **チェックの分け方**: `step2_check.STATUS_WARN`・`is_blocking`・`classify_checks`。止める＝C2・D1 の NaN・B1。他の NG は注意で yaml を書く（終了コード 0）。C2 は書いた後に確かめ、NG なら書く前の yaml に戻して終了コード 8。前の pkl は消さない。

#### 10-2・10-3 結果（2026-11-16）
- 10-2 実機 tests 手順13（写しの yaml）: 4 点とも帯の基準で保てた（7.41〜8.03%）。瞬間の判定も 4 点で計算（瞬間 2〜4）。tests の yaml の md5 不変。瞬間だけで合格する場面は実機では出ず、11-16 の CSV の机上で確認済み。
- 10-3 取り込み: port_research で 77 ファイル（新規 hold_crossing.py・差 5 本）。ruff・tests/unit 339 件通過。等価確認① は `feedforward.stop_brake_offset_pct`（今朝の学習で 14.21、tests 13.89）だけ不一致。等価確認②（スタブ、写しの車で 13.89 に戻して tests `1,3` と src `1,24`、60 s）: 指令の差 p95 アクセル 0.003%・ブレーキ 0.001%（最大 1.48% は切り替えの 1 行）。

### 段11（2026-11-16 ユーザー決定）: 画面の見直し・一時停止
- 全体: 1 ページに収める（画面の大きさに合わせて伸縮。1920×1080 と 1280×800 で確認）。説明文・「手順○○」の表示を消す。
- キャリブレーション: max も測る（ジョグ →「ここを max として保存」）。max を測るときだけストロークの端（100%）の手前まで動かせる。
- 車両プロファイル: 1 ページ。ボタンの設定（有無を含む）は全社共通（共通設定）へ移す。出す項目: 基本（名前・最高速・最大減速 G）／ペダル min・max（キャリブレーションの値・手で直せる）／停止判定（偏差しきい値・継続秒）／運転指標用（質量・A0・A1・A2）／学習の結果（日時・使っているモデル・クリープ車速・停車保持の開度。読むだけ）。
- 自動運転: 4 段の高さ 1:1:0.5:0.5。ペダルの図にボタン（スタート・P・N・D）を押していた時間の矩形で出す（ペダルの縦軸 0〜100% と同じスケールで **0 = 押していない・50 = 押している**。押し始めで 50 に立ち、押した秒数だけ 50 で横に伸び、離して 0 に落ちる。高さはどのボタンも同じ 50・ボタンごとに色を分け、凡例にスタート・P・N・D）。最下段の真ん中は 1 行: 状態 ｜ 区間 ｜ 経過/全体 ｜ 最大|偏差| ｜ p95。走行中のボタンは「一時停止」「走行終了」（非常停止ボタンは出さない）。
- 一時停止: 押された時点の車速を保つ。モードの時刻を止め、再開で止めた時刻から続ける。一時停止中の行は印を付け、KPI から外す。tests で作って試してから src に取り込む。
- 学習運転: 自動運転と同じレイアウト（一時停止なし）。
- ログ: 合否を消す。運転指標 EER・IWR・ASCR を出す。tests の走行レポートのように偏差を出す（偏差の図・区間別の表。基準の線・合否は出さない）。

#### 段11 実装結果（sonnet、2026-11-16。画面・API・ランナーのイベントまで。一時停止のランナー側（tests で作ってから取り込む）は別担当で未実装）
- **ボタンの設定を共通へ**: `buttons:` 全部（秒数・servo_map・`installed` ほか）を `config/robot_common.yaml` へ移した（値は今の車両 yaml と同じ・`installed: false`）。`config_store` は `VEHICLE_SECTIONS = ("vehicle",)`・`IGNORED_VEHICLE_SECTIONS = ("buttons",)`。**車両 yaml に `buttons:` が残っていても無視（共通の値を使う）して警告を 1 行出す**（読めなくはしない。上書き案より安全: 車ごとに違う値がこっそり効かない・移行の途中で壊れない）。車両プロファイル API・画面からボタンの項目と servo_map の書き換え（`rewrite_servo_map_text`・`test_vehicle_servo_map.py`）は外した。`ProfileResponse.buttons` は共通設定の値を読むだけ。
- **キャリブレーションで max**: 画面に「ここを max として保存」・`POST /drive/calib/save-max {axis}`・ランナーの対話コマンド `save_max`（今の実位置 → `vehicle.max_*_opening_pct`。min+1% 以上・99% 以下でなければ NG）。`/calib/jog` は `calib:true` を付け、ランナーの上限をストロークの端 100% の手前 **99%**（`CALIB_STROKE_LIMIT_PCT`。1% ＝ 約 0.95 mm の余裕で端当てを避ける）まで広げる。手動運転は今までどおり max まで。片足ルールは同じ。
- **車両プロファイル（編集）**: 1 ページ（基本／ペダル min・max／停止判定／運転指標用／学習の結果（読むだけ。日時・モデル名・クリープ車速・停車保持の開度））。
- **自動運転・学習の共通レイアウト**（`DriveMonitorScreen`）: 4 段 1:1:0.5:0.5・窓の大きさに合わせて伸縮（1920×1080・1280×800 でページのはみ出しなし）。ペダルの図にボタンの矩形（**0 = 押していない・50 = 押している**・色: スタート紫／P 黄／N 緑／D 桃・凡例つき・押している最中は現在時刻まで伸ばす）。最下段の真ん中は 1 行 `状態 ｜ 区間 ｜ 経過/全体 ｜ 最大|偏差| ｜ p95`。ボタンは「一時停止」（押すと「再開」・モード走行のときだけ）と「走行終了」。非常停止ボタンは外した。サーバーログのターミナル表示（現在ステータス）は外した（ログページの「メッセージ」で見られる）。
- **ランナー・JobManager**: `patches.install_button_events`（差し替え 9。最も外側の `press_button` が `button` イベントの start と、seconds 後の end を出す。ボタンなしでも出る）。JobManager は最新 sample の `mode_time_s ?? t_s`（sample 前は 0）で `button_events` を持ち、`paused`・`dev_stats`（モード走行の |偏差| の最大・p95。一時停止中・`paused` 印の行は除く）とともに WS の `job` に出す。`sample` に `pattern`（区間名）を足した。
- **一時停止 API**: `POST /drive/pause`・`/resume` → ランナーへ SIGUSR1 / SIGUSR2（`JobManager.pause/resume`・`paused` を持つ）。モード走行以外・状態違いは 409（日本語）。**ランナー側は未実装**: `run.py` に「受けたと言って無視」する受け口だけ置いた（既定動作＝プロセス終了を防ぐ）。実装するときはこの受け口を置き換える。ランナーが `state` イベントに `paused` を付ければ JobManager がそれに合わせる。
- **ログ**: 合否を消した。左に一覧・右にタブ（走行グラフ／運転指標・偏差／ログ表／メッセージ）。`GET /sessions/{id}/analysis`（モード走行のセッションだけ。`src/web/analysis.py`: EER・IWR・ASCR は core の `compute_drive_index`、質量・走行抵抗は車の `vehicle.*`、偏差の KPI（最大・p95・>0.5 s・>1 s・平均）・偏差の図・WLTP 区間別の表）。tests の走行レポート（`drive_log_real_20261116_082505` 由来の偽セッション）と一致: 最大 0.48・p95 0.21・平均 −0.01・EER +0.53・IWR −0.80・ASCR +0.04。
- **小さな修正**: `devices.ups` が false のとき右上の「UPS AC OK」を出さない。「手順○○」の表示・説明文は画面から外した（ホームの各段の説明・初期化の説明・手動運転の注意・キャリブレーションの手順文 ほか）。
- **確認**: ruff（`src tests/unit`）通過・tests/unit 347 件通過（`test_pause_and_buttons.py`・`test_interactive_calib.py` を追加、`test_buttons_off.py`・`test_vehicle_store.py`・`test_robot_config_store.py` を更新、`test_vehicle_servo_map.py` を削除）。スタブの uvicorn（8081・車フォルダ複製・DB なし）で全画面を Playwright で確認（`.playwright-mcp/s11_*.png` ＝ 1920×1080、`s11_1280_*.png` ＝ 1280×800）。走行中の自動運転の画面は WebSocket を差し替えた偽データで確認（実機が学習走行中のためランナーは起動していない）。
- **車両 yaml の移行（ユーザーが後で行う）**: `vehicles/test_vehicle/vehicle.yaml` の末尾 `buttons:` 節（`# ── スタートスイッチ…` のコメントごと）を削除。残していても無視されるので急がない。


#### 一時停止（tests）実装結果（sonnet、2026-11-16。tests/research のみ・src は未変更）
- **仕様（ユーザー決定）**: 一時停止＝押された時点の車速を保つ。モードの時刻を止め、再開で止めた時刻から続ける。SIGUSR1＝一時停止・SIGUSR2＝再開（ハンドラはフラグを立てるだけ・制御ループが毎周期読む）。手順 3・24 の `_ModeRun`（`mode_drive.py`）に入れた。シグナルを受けなければ今までと同じ。
- **作り**:
  - 一時停止が来たら、押された時点の CAN 車速 v（停車速度以下なら 0＝停車保持）を読み、**定速 v の DrivingMode（`make_pause_mode`）で FF 候補を作り直した別の `_ModeRun`** を別スレッドで作る（`default_pause_setup`＝`prepare_mode_drive_from`・表示は捨てる。約 0.25 秒）。作る間も本体のモードは走り続け（ペダルを止めない）、できた周期で切り替える。
  - 切り替えた周期のモードの時刻 t_p で本体の時刻を止め、止めている間はその `_ModeRun._cycle` を回す。基準＝v の定速・FF＝その表の定速の開度（要る加速度 0）・FB（C10 の P・I・DOB）は v からの偏差で本体と同じ部品が働く。調停・ガバナー・実開度の安全網（片足ルール）・過電流/アラーム/偏差の安全網・ペダル指令の状態は本体と共有（周期ごとに受け渡し）。DOB・C10 の I は一時停止の制御の側で 0 から始まる。
  - 再開で一時停止の制御を捨て、本体は t_p の周期から続ける（本体の窓・状態は一時停止の間は触らないので途切れない）。モードの時刻は `pause_total` ぶん引き、調停・ガバナーの時刻（now）は実時間のまま連続させる（ボタンの D 待ちの `hold_total` と別に持つ）。
  - 保つ車速に「その時刻の基準」でなく「押された時点の実車速」を選んだ理由: ユーザー決定どおり、かつ偏差の取れていた実車速のほうが一時停止に入る瞬間の段差が出ない。
  - 手順 24 は D を押し終えるまで一時停止を受け付けない（無視して表示）。`say()` は受付・切り替え・解除に出す（モードの時刻・車速・基準・止めた秒数）。
- **ログ**: 一時停止の行は `section` が `MODE_DRIVE_PAUSE`（元の section＋`_PAUSE`）・新列 `paused`（末尾。0/1）・`mode_time_s` は止めた時刻・`ref_speed_kmh` は保つ車速。`MODE_DRIVE` だけ数える `rows_from_samples`・`rows_from_csv`・KPI・レポート・compare_runs 系から自動で外れる。レポートの注記と `ModeDriveResult.paused_s／pause_count` に回数と秒数を出す。
- **変更ファイル**: `tests/research/mode_drive.py`（`_ModeRun` の一時停止・`run_mode_drive(pause_setup_factory=)`・`ModeDriveResult`）・`drive_log.py`（`paused` 列）・`main.py`（レポートの注記）・`test_research_pause.py`（新規 8 件）・`test_research_drive_log.py`（列の末尾位置の更新と paused 列の 1 件）。yaml は未変更。
- **スタブ確認**（`--hw stub --steps 1,3 --limit-s 120`・WLTP 先頭 120 秒・モードの時刻 約 40 秒で SIGUSR1・15 秒後に SIGUSR2。スタブの車両は実車と別物なので制御ゲインの判断には使わない）:
  - モードの時刻 40.30 s（押されたのは 40.05 s・切り替えまで 0.25 秒）で止まり、止めていた 295 周期（14.7 秒）の `mode_time_s` は 40.3 のまま、基準は 34.503 km/h 一定。再開後 40.30 s から続いて 119.95 s（120 s）まで完走（モード 2400 行・単調増加・一時停止 295 行は別 section）。
  - 一時停止中の車速は 33.7〜37.1 km/h（切り替え直後の 3 秒の整定を除くと 35.2〜37.1・偏差 +0.7〜+2.6）。保つ側が 9.228%（C10 の遊びの下限＝効き始めの手前で止まる値）に張り付いて浅くできず、スタブでは 0.16 km/h/s で上へずれた。実車の保持の効きは別なので数字を実機で見る。
  - KPI・レポート: 「判定は MODE_DRIVE の行（2400 行）」で、一時停止 295 行は入らない。最大|偏差| 3.27（再開後に保持中のずれ＋基準の減速が重なった分）。レポートの注記に「一時停止 1 回・計 14.8 秒」。
  - 受信なしの確認: 同じ条件（`--limit-s 60`）を実装前後で走らせ、同じモードの時刻で比べた（スタブは壁時計で動くのでビット一致にはならない）: ペダル指令の差の平均 アクセル 0.026%・ブレーキ 0.005%・車速 0.013 km/h（最大 2.0%・0.24 km/h は切り替えの周期）。`paused` は全行 0・既存の列の並びは同じ（末尾に 1 列）。
- **テスト**: `test_research_pause.py` 8 件通過（受信なし＝今までと同じ／凍結・保つ基準・再開で時刻が飛ばない・KPI 入力から除外・CSV の paused／再開だけは無視／切り替えを待つ間もペダルが止まらない／保つモードの長さを超える一時停止／D 前は無視）。`test_research_drive_log.py` は列の更新後 1 件を除き通過（`test_steps_1_2_...` は `main.py:483 'NoneType' ... 'checks'` で、この変更と無関係）。`ruff check tests/research` 通過。`test_research_mode_drive.py` の 46 件失敗は yaml の既定（hold_pi・stop_tail など）と古いテストの食い違いで、この変更の前から出る（`ValueError: stop_tail_kmh には…`・`ConfigError: hold_pi_enabled は…`）。
- **未解決・注意**:
  - 切り替えまでの 0.25 秒の間に動く量と、保持中のずれ（上の 9.2% 張り付き）が再開の偏差になる。再開直後は一時停止の制御から本体へ替わるので、ペダルが 1 周期で向きを替えることがある（片足ルール・実開度の安全網は働く）。実機で保持の整定とずれ幅を見て、必要なら保つ車速の選び方（押された時／切り替え時）を決める。
  - DOB・C10 の I は一時停止の制御で 0 から始まる（本体の値は引き継がない）。長い一時停止のほか、定速モードの長さ 40 秒を超えたら時刻を巻き戻す（窓を捨てる。テスト済み）。
  - 実機の確認（`--hw real`）は未実施（実機走行中のため）。WebUI のランナー（src）への取り込みは `scripts/port_research.py` で後で行う。


#### 一時停止 直し（実機 1 回目の結果から）（sonnet、2026-11-16。tests/research のみ・src 未変更）
- **実機 1 回目の結果**（WLTP・モードの時刻 約 40.15 s で SIGUSR1・15 秒後に SIGUSR2）: 保つ制御を作るのに 2.7 秒（スタブ 0.25 秒）かかり、その間モードの時刻が進んで（40.15→42.85）減速する基準に追従し 24.7 km/h まで落ちた。切り替え後に 34.28 へ戻す途中で 36.5 まで行き過ぎ、再開は時刻 42.85（基準 25.5）から始まって再開後の最大|偏差| 9.84 km/h。
- **遅さの原因**（測った）: 作る定速モードの長さに比例し、**車速によって 16 倍違う**。表の「アクセルの戻し・踏み増し」の解き直しが、車速によって全行（定速でも）で走るため（20.0・20.37・24.07 km/h などは 40 秒で 約 1.2〜2.3 秒、20.74・21.11 などは 0.14 秒。約 3 割の車速が遅い側）。実機は 別スレッドで GIL を本体の周期と取り合って 2.7 秒に伸びた。
- **直し**:
  1. 別スレッド・切り替え待ちをやめ、**押された周期の中で作る**。その周期でモードの時刻 t_p を止める（受け付けた周期＝t_p。作る間に進むことがない）。作る間に別の制御で走る区間は無い（その周期のペダル指令は前の周期のまま・本体のループが数周期遅れるだけ）。作るのにかかった秒数は `[一時停止] 受け付けました…` の行に出す。
  2. 保つ定速モードの長さ 40→**6 秒**（`PAUSE_PLAN_S`）。作る時間は 最悪の車速でも 約 0.35 秒（40 秒は 2.3 秒）・多くは 0.02 秒。モードの外に出る長い一時停止は、開度の列も基準も最後の値のまま（定速なので同じ。巻き戻しをやめた＝窓を捨てない）。
  3. 再開は t_p（基準 ≒ 押された時の車速）から続く。一時停止の行（section `MODE_DRIVE_PAUSE`・`paused=1`）は押された周期から。
- **スタブ**（`--steps 1,3 --limit-s 120`・40.05 s で SIGUSR1・15 秒後に SIGUSR2）: 作るのに 0.03 秒。押された周期 40.05 s（車速 34.5 km/h・基準 34.48）の次の周期から一時停止の行・保持中の車速 34.4〜37.3（スタブは保持側が上へずれる別物の車両・押した直後の 5 周期は 34.4〜34.5。前回と同じ 0.16 km/h/s のずれ）。再開は 40.05 s から続き（行の間隔 0.053 s）、再開後の最大|偏差| 2.88 km/h（保持中にずれた分。実機の 9.84 から）。
- **受信なしは同じ**: `--steps 1,3 --limit-s 60` を直す前の基準と比べ、同じモードの時刻でペダル指令の差の平均 アクセル 0.010%・ブレーキ 0.000%・車速 0.001 km/h（最大 1.49%・0.018 km/h）。`paused` は全行 0。
- **テスト**: `test_research_pause.py` 9 件通過（押された周期で時刻が止まる＝制御を作るのに 0.5 秒かけても進まない・再開は止めた時刻から・作る間の行も paused・どの車速でも作るのが 1 秒未満）。KPI 除外・手順 24 の D 前の無視・片足ルール・安全網は変更なし。`ruff check tests/research` 通過。
- **実機で見ること**: 押した直後に保持の開度へ替わる周期のペダルの段差（減速中のブレーキ→保持の開度）と、保持中のずれ・再開後の最大|偏差|。

#### ログの 3 段の図（sonnet、2026-11-16）
- **画面**: ログの「走行グラフ」を、試験の走行レポートと同じ 3 段（x 軸共有・高さ 1 : 1 : 0.5）に変えた。①車速（基準＝破線・実車速）②ペダル開度 0〜100%（アクセル・ブレーキ＋ボタン押下の矩形。押していない 0・押している 50。色・凡例は自動運転画面と同じ。`auto-drive.js` が `window.BUTTON_COLORS/LABELS/PRESSED_LEVEL` を出し、`logs.js` が読む）③偏差（実車速 − 基準・基準の線なし）。x はモード走行なら「モード時間 [s]」、学習などは「経過時間 [s]」。ズーム・パンは無い（3 段とも同じ x）。「運転指標・偏差」タブからは偏差の図を外し、KPI の行と 2 つの表だけにした。サイクルを選んだときの図（複数セッションをつないだ 2 段）は従来どおり。
- **API**: `GET /sessions/{id}/analysis` に `figure`（`x_label`・`t`・`ref`・`act`・`accel`・`brake`・`buttons[{button,seconds,start_s,end_s}]`）を足した。auto 以外も `{"available": false, "figure": …}` を返す。`chart`（旧・偏差の図の点列）は削除。`src/web/analysis.py` の `build_figure`。
- **ボタンの保存（今後）**: JobManager がボタンの `start` イベントを受けるたびに、`drive_session_messages` へ level `BUTTON`・message `{"button": "N", "seconds": 1.2}`（time＝押し始めの壁時計）の行を 1 本足す（スキーマ変更なし）。`GET /sessions/{id}/messages` は BUTTON の行を出さない（同じ押下の通常の文が別にあるため）。
- **ボタンの読み直し（過去のセッション）**: BUTTON の行が無ければ、メッセージの `ボタンなし: 「N」を押したことにします（1.2s 待ちます）`／`ボタン「D」を ch1 175° で 1.2s 押します …` を正規表現で読み（スタート→start）、その time（押し始め）を、ログの timestamp ↔ x の対応で補間して x にする。ログの範囲外は傾き 1 で延ばす。`src/infra/button_events.py`（app と web の両方から使うので infra に置いた）。
- **確認**: ruff（`src tests/unit`）通過。`tests/unit/test_log_figure.py`（新規 8 件）・`test_interactive_calib.py`（`chart` 削除に追従）ほか関連 187 件通過。8081 のスタブ uvicorn（DB は本番を読むだけ）で 2026-11-16 の auto 74b3ebda（スタート・D が x=0 付近）と学習 a1d5cdd5 を Playwright で確認し、停止済み。スクショ: `.playwright-mcp/s11b_logs_{auto,learning,analysis}_{1920,1280}.png`（1280×800 でもはみ出しなし）。
- **未解決**: 8080 の実機サーバーは再起動するまで旧 API（`figure` なし）なので、新しい図は再起動後に出る（それまで「図にできるログの行がありません」）。BUTTON 行の書き込みは単体テストのみ（実走行は再起動後）。モード走行の前後（0 秒より前のスタート短押し、終了後の N・IGN-OFF）は x が範囲外で描かない。

#### 段11 実機確認（2026-11-16 午後）
- 新しいモデル（`fwd_model_20261116_133627.pkl`、手順2 の 3 回目。4 点は帯で 0/4・瞬間で 4/4 だが hold_pi が求まらず前の pkl から引き継ぎ・注意 H1 の 1 項目でモデルを使った）で画面から WLTP: 終了コード 0・最大 0.68・p95 0.22・|偏差|>0.5 3.00 s・>1 0 s・往復 1.04（同じ日の tests 0.56・0.23・0.35 s・0・1.12）。BUTTON の行が DB に 4 本。
- 学習の終わりの N で落ちていた（終了コード 1）: 手順2 は保存後に設定を読み直し、読み直した設定に src 専用の値が付かないため ボタンなし が効かなかった → patches は configure で起動時に決めた値を使う（テスト追加）。
- 一時停止: tests 実機 1 回目は保持の制御を作るのに 2.7 s かかり（別スレッドと GIL の取り合い）、その間モードが進んで再開後に最大 9.84 km/h → 押した周期で時刻を止め・押した時刻から再開・保持のモードを 6 s に短縮（作る時間 0.03 s）。tests 実機 2 回目: 保持 33.18〜33.81 km/h（目標 33.67）・再開後の最大 0.71 km/h。src へ取り込み（port_research）、ランナーはモード走行以外の SIGUSR1/2 を無視（core が外した後も無視に戻す）。画面から実機: 41.26 s で一時停止・9.2 s 保持（30.5〜31.6 km/h）・再開後の最大 1.25 km/h・終了コード 0。
- キャリブレーションの max: 画面のジョグ（キー操作）でブレーキ 47.3%・アクセル 61.0% にして「ここを max として保存」→ yaml 47.3・61.0（tests と同じ）。47.3% は pulse にすると半端（4494 = 47.305%）で 47.31 と保存された → 保存する min・max をジョグの 0.1% の目盛りに丸める（テスト更新・追加）。アクセル 61% の間に模擬車速は 145 km/h まで上がった。
- 画面の読み直しで軌跡が消える件: サーバーが最近 120 s の値を覚え `GET /api/v1/drive/history` で返し、画面は接続時に空なら埋める（ユーザー承認）。
