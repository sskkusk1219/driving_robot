# 実行コードのメモ
- ユーザーが実行するコードのメモです。
- 議論内容や設計内容はここに記載しないでください。

sudo shutdown -h now
vcgencmd measure_temp

cd driving_robot
source .venv/bin/activate
ssh raspi5_16gb@raspi5-16gb.local

## 非常停止スイッチの原点復帰確認
.venv/bin/python tests/hardware/test_emergency_stop_home_return.py
## 過電流の原点復帰確認
.venv/bin/python tests/hardware/test_overcurrent_home_return.py
## キャリブレーションの確認
.venv/bin/python tests/hardware/test_calibration.py
  操作方法:
  - e: +0.5mm（前進）
  - w: -0.5mm（後退）
  - d: +0.1mm（前進）
  - s: -0.1mm（後退）
  - Enter: 確定
  - q: 中止

## CANの受信確認
sudo .venv/bin/python scripts/check_can.py
python /home/raspi5_16gb/projects/driving_robot/scripts/check_can_2.py

## UPSの確認
.venv/bin/python tests/hardware/test_ups_monitor.py
  出力の見方:

  チェック1 — NUT 通信確認（一瞬で完了）
  NUT socket 接続      : OK
  battery.charge (raw) : 095.0
  ups.status (raw)     : OL

  チェック2 — バッテリー残量表示
  バッテリー残量 : 95.0%  [███████████████████░]
  UPS ステータス : OL
  AC 通電中      : YES

  チェック3 — AC断コールバック監視（Ctrl+C まで継続）
  [   0s] OL      95.0%  OL（AC通電中）
  ここで UPS の AC ケーブルを抜くと:
  ★★★ AC断コールバック発火！（1回目）★★★
      ups.status が OL → OB に遷移しました
      → SafetyMonitor.handle_ac_power_loss() が呼ばれます
  ケーブルを戻すと OB→OL になりますが、コールバックは発火しません（立ち上がりエッジのみ）。

## SG90の動作確認
  .venv/bin/python tests/hardware/test_servo_pca9685.py [ch]
  @20260701-servo-pca9685-smoke-test
  
## 本番環境の起動
  初回のみ: DB初期化

  cd /home/raspi5_16gb/projects/driving_robot
  DATABASE_URL=postgresql://localhost/driving_robot \
    .venv/bin/python scripts/setup_db.py
　→済
　確認方法
　psql driving_robot -c "\dt"

  サーバー起動

  cd /home/raspi5_16gb/projects/driving_robot

  ## ハードあり本番環境
  DRIVING_ROBOT_USE_REAL_HW=1 \
  DATABASE_URL=postgresql://localhost/driving_robot \
    .venv/bin/uvicorn src.web.app:app --host 0.0.0.0 --port 8080

  再起動なしでも現行サーバに対し Ctrl+Shift+R
  すれば（ディスクのJSは既に修正済みなので）動作します。再起動は恒久対策を効かせるためのもの

  ## ベンチモード
  DRIVING_ROBOT_USE_REAL_HW=1 DRIVING_ROBOT_BENCH_GPIO_ONLY=1 \
    DATABASE_URL=postgresql://localhost/driving_robot \
    .venv/bin/uvicorn src.web.app:app --host 0.0.0.0 --port 8080

  GUI アクセス

  同一LANのブラウザから: http://<RaspberryPi のIP>:8080
  http://100.64.1.33:8080

  raspiのIP確認方法
  hostname -I
  10.155.61.20 2400:2200:436:9f78:459a:47bb:38f0:93c

  ---
  環境変数の意味

  変数: DRIVING_ROBOT_USE_REAL_HW
  値: 1       
  省略時の挙動: スタブ（モック）で起動。HW接続なしで動作するが GPIO・Modbus・CAN は全て無効
  ────────────────────────────────────────
  変数: DATABASE_URL
  値: postgresql://localhost/driving_robot
  省略時の挙動: in-memory DB で起動。再起動するとプロファイルなどは消える

  ---
  確認: スタブモードで動作確認（ハード不要）
  
  実機を繋がなくても以下でサーバーが起動し、GUIの動作確認ができます:

  cd /home/raspi5_16gb/projects/driving_robot

  ## HW なし・DB なし（最小構成）
  .venv/bin/uvicorn src.web.app:app --host 0.0.0.0 --port 8080

  ## HW なし・DB あり（プロファイルを保存したい場合）
  DATABASE_URL=postgresql://localhost/driving_robot \
    .venv/bin/uvicorn src.web.app:app --host 0.0.0.0 --port 8080

  ## 使用中のプロセスを確認
  lsof -i :8080

  既存プロセスを止めてから再起動する場合:

  ## プロセスを終了
  kill $(lsof -ti :8080)

  ---
  注意点
  
  現状 start.sh は存在しません（scripts/ には setup_db.py と check_can.py のみ）。毎回上記コマンドを手動実行する必要があります。

  systemd サービス化するか、start.sh を作るかは追加作業になります。必要なら対応できます。

source .venv/bin/activate
deactivate

claude update
claude --version

## `docs/Problem/ProblemReport_20260910.md`の実行コード
### 車速追従性を検証するコード
  `cd /home/raspi5_16gb/projects/driving_robot`

  # 手順一覧
  `.venv/bin/python -m tests.research.main --list`

  # 手順 0 を実行（設定の検証・要約表示）
  `.venv/bin/python -m tests.research.main --upto 0`

   ## 未実装の手順に当たると明示停止することの確認（exit=3）
   `.venv/bin/python -m tests.research.main --only 3; echo "exit=$?"`

   ## 設定を壊したときにちゃんと止まるかの確認（max_decel_g を 2.5 などにして実行）
   `$EDITOR tests/research/config_testVehicle.yaml`

   ##  テスト
   `.venv/bin/python -m pytest tests/research -q`

  # 手順 1 を実行（初期化: サーボ通信 / エラー消去 / サーボON / CAN / UPS / 原点復帰）
   ## スタブで流れだけ確認（実機に触らない。CI・普段の確認はこちら）
   `.venv/bin/python -m tests.research.main --only 1`

   ## 手順 0 → 1 を通す
   `.venv/bin/python -m tests.research.main --upto 1`

   ## 実機で初期化する（**アクチュエータが物理的に動く**。周囲の安全を確認してから）
   ##   前提: config/settings.toml がある / USB-RS485 2本・Kvaser・NUT(upsd) が生きている
   `.venv/bin/python -m tests.research.main --upto 1 --hw real`

   ## 終了コード: 0 成功 / 2 設定エラー / 3 未実装の手順 / 4 初期化エラー
   `echo "exit=$?"`

  # 手順 2 を実行（ペダル探索 → 閉ループパターン走行 → 2次多項式 FF モデル作成）
   ## 走行はハードウェアを使うので、手順 1（初期化）と一緒に実行する
   ## 実行順: 手順 1（初期化）→ 走行前チェック → 手順 2
   ##   走行前チェック: 本番と同じ項目（通信・サーボ・UPS・アクチュエータ位置）を確認 →
   ##   ブレーキを 0.5mm ずつ踏み、1s ごとの平均車速が 0.02 km/h 未満になったら停止確認 → 車速 0 km/h を含めて再確認
   ## 手順 2 の最初にペダル探索: クリープ中に 0.5mm ずつ踏み、アクセル/ブレーキの不感帯と停止確認開度を測り、
   ##   停止確認開度 +10% で停車保持してから走り出す（刻み・待ち・判定は pedal_search セクションで変更可）
   ## スタブで流れを確認（実機に触らない。config_testVehicle.yaml は書き換えない）
   `.venv/bin/python -m tests.research.main --steps 1,2; echo "exit=$?"`

   ## 実機で走行する（**シャシダイナモ上で。車両が 0→約137 km/h まで加減速する**）
   ##   前提: 手順 1 の前提 + 両ペダルを離すと車両がクリープで動くこと（ペダル探索で使う）
   ##   開度は 原点 = 0% / 9500 pulse = 100%（キャリブレーションは使わない）
   ##   ペダル探索の 3 値（不感帯 2 つ・停車保持開度）は走行前に、
   ##   feedforward.model_path と物理定数は走行後に config_testVehicle.yaml へ保存される
   `.venv/bin/python -m tests.research.main --steps 1,2 --hw real; echo "exit=$?"`

   ## 走行後: アクセル 0% → ブレーキ不感帯の手前まで一発 → 0.5mm 刻み・1s 待ちで 0.2G 付近の緩減速 → 停車保持
   ##   （0.18G 未満で踏み増し・0.3G 超えのときだけ戻す。値は decel_stop セクションで変更可）

   ## 走行グラフ（SSH 接続で画面が無いとき）: 2s ごとに更新される PNG を開く
   `tests/research/results/live_drive.png`
   ## 走行ログと図（走行前チェック〜停車保持を 1 本。section 列で区間を分ける）:
   ##   tests/research/results/drive_log_<stub|real>_<日時>.csv / .png

   ## 終了コード: 5 走行エラー（ペダル探索の失敗・非常停止・タイムアウト） / 6 モデル作成エラー（サンプル不足）
   ##            / 7 走行前チェックエラー（NG 項目あり・ブレーキを上限まで踏んでも停車しない）

  # 手順 3 を実行（FF のみで WLTP モード走行 → CSV / 図 / レポート）
   ## 実行順: 手順 1（初期化）→ 準備（走行モードを DB から・FF モデルを読む）→ 走行前チェック → 手順 3
   ##   準備に失敗したら HW に触らず終了コード 2（DB に繋がらない・モード名が無い・FF モデルが無い）
   ##   制御: FF（手順 2 のモデル）→ effort の符号でアクセル/ブレーキに振り分け（Kp=Ki=Kd=0、
   ##         不感帯補償・レートリミット・ヒステリシスなし）
   ## スタブで流れを確認（実機に触らない）。--limit-s でモードの先頭だけ走る（1800s 全部は 30 分かかる）
   `.venv/bin/python -m tests.research.main --steps 1,3 --limit-s 60; echo "exit=$?"`

   ## 実機で走行する（**シャシダイナモ上で。WLTP 1800s・最高 131 km/h。走行前チェック込みで約 31 分**）
   ##   前提: 手順 2 が済んで config_testVehicle.yaml の feedforward.model_path にモデルがある / PostgreSQL が動いている
   ##   減速G ガバナー（安全網）: 実測減速度が 0.4G × 0.98 以上ならブレーキを頭打ち（mode_drive セクションで無効化可）
   `.venv/bin/python -m tests.research.main --steps 1,3 --hw real; echo "exit=$?"`

   ## 走行中: ターミナルに 1s ごと（基準車速・実車速・偏差・Kp/Ki/Kd・アクセル・ブレーキ・FF 出力・区間・最大偏差・進捗）
   ##   グラフ（SSH で画面が無いとき）: tests/research/results/live_drive.png（2s ごとに更新）
   ## 結果:
   ##   走行ログ CSV（0.1s 刻み）: tests/research/results/drive_log_<stub|real>_<日時>.csv（section=MODE_DRIVE の行がモード走行）
   ##   レポート: tests/research/results/reportYYYYMMDD_RunFF.md ＋ 図 tests/research/results/reportYYYYMMDD_RunFF/
   ##   （同じ日の 2 本目以降は _2, _3 …）

   ## レポートを CSV から作り直す（走行後にレポート作成だけ失敗したとき・しきい値を変えて見直すとき）
   `.venv/bin/python -m tests.research.mode_report tests/research/results/drive_log_real_<日時>.csv --label FF`

   ## 終了コード: 5 走行エラー（CAN・軸通信・過電流・最高速超え・逸脱で中断。ペダルを離してから途中までのレポートを作る）

## `docs/Problem/ProblemReport_20260912.md`の実行コード
### 1. モード走行時の FF パラメータ理解（車両には触らない。PostgreSQL は必要）
  `cd /home/raspi5_16gb/projects/driving_robot`

  # 解析（表をターミナルに出し、図を tests/research/results/reportYYYYMMDD_explanationFF/ に保存。約 30 秒）
  ##   A: WLTP の基準車速だけを FF に通す / B: 手順 3 の走行 CSV を FF の分岐ごとに集計 / C: 各パラメータの感度
  ##   D: 不感帯のせいで実質惰行になる要求加速度の範囲 / E: 定速に要るアクセル開度（同定値から計算）
  `.venv/bin/python -m tests.research.ff_explain --csv tests/research/results/drive_log_real_20260911_171637.csv`

  ## 拡大図の区間を変える（開始:終了 [s] をカンマ区切り）
  `.venv/bin/python -m tests.research.ff_explain --csv tests/research/results/drive_log_real_20260911_171637.csv --zoom 20:120,860:927`

  ## テスト
  `.venv/bin/python -m pytest tests/research/test_research_ff_explain.py -q`

  ## レポート: tests/research/results/report20260912_explanationFF.md

### 2. 2次多項式 Ridge 逆モデルの分析（車両には触らない）
  `cd /home/raspi5_16gb/projects/driving_robot`

  # 2-1 手順 2 時点の当てはまり（R²・RMSE・MAE）。約 10 秒
  ##   学習セットを作り直して保存済みの指標を再現できるか確かめてから（できなければ終了コード 2）、
  ##   パターン単位の分割検証・ラベル別・パターン種別ごとの誤差を出す。図は results/reportYYYYMMDD_analysisFF_model/
  `.venv/bin/python -m tests.research.model_analysis --part metrics; echo "exit=$?"`

  # 2-2 モデルだけで WLTP 1800s を通す（PostgreSQL が必要。約 30 秒）
  ##   2 通りの開度を CSV（time_s, ref_speed_kmh, accel_opening, brake_opening）と PNG で出す:
  ##     wltp_model_raw … モデル出力そのもの（1.0s 先の車速変化 ≥ 0 ならアクセルモデル、< 0 ならブレーキモデル）
  ##     wltp_ff        … 手順 3 と同じ FF 指令（停車保持・惰行テーパなどの定数込み）
  ##   保存先 results/reportYYYYMMDD_analysisFF_model/（比較の拡大図 compare_*.png も）
  `.venv/bin/python -m tests.research.model_analysis --part simulate; echo "exit=$?"`

  ## 拡大図の区間を変える（開始:終了 [s] をカンマ区切り）
  `.venv/bin/python -m tests.research.model_analysis --part simulate --zoom 20:120,860:930`

  ## 学習 CSV を指定する（別のモデルを分析するとき。config の feedforward.model_path と対にする）
  `.venv/bin/python -m tests.research.model_analysis --part metrics --train-csv tests/research/results/drive_log_real_XXXX.csv`

  ## テスト
  `.venv/bin/python -m pytest tests/research/test_research_model_analysis.py -q`

  ## レポート: tests/research/results/report20260912_analysisFF_model.md

### 3. 手順 2・手順 3 の改善提案（車両には触らない）
  `cd /home/raspi5_16gb/projects/driving_robot`

  # 段階 1 簡易車両モデルの確認（PostgreSQL が必要。約 3 分）
  ##   K-1 ペダル応答の表（手順 2・手順 3 の実走行ログから同定）と config のペダルゲイン比例の比較
  ##   K-2 開ループ再生: 実走行の指令開度だけで 5s 走らせた車速の誤差（1s ごとに実車速から再開）。
  ##       車両モデル × むだ時間 × 一次遅れ を総当たりし、手順 3 のログで誤差が最小の組を選ぶ
  ##   K-3 閉ループ再現: 手順 3 と同じ FF 指令で WLTP を模擬し、実走行（926s で中断）と比べる
  ##   図は results/reportYYYYMMDD_KAIZEN_process2,3/（simcheck_overview.png・simcheck_20_120.png・simcheck_860_930.png）
  `.venv/bin/python -m tests.research.kaizen --part sim-check; echo "exit=$?"`

  ## 別の走行ログで確かめる（--run-csv は手順 3 のモード走行、--train-csv は手順 2 のパターン走行）
  `.venv/bin/python -m tests.research.kaizen --part sim-check --run-csv tests/research/results/drive_log_real_XXXX.csv`

  ## テスト
  `.venv/bin/python -m pytest tests/research/test_research_vehicle_sim.py tests/research/test_research_kaizen.py -q`

  # 段階 2 FF 改善案の比較（PostgreSQL が必要。約 7 分）
  ##   S-1 学習の中身（現行 / 効いている行だけ / 0.5s ずらし）
  ##   S-2 WLTP 1800s の閉ループ模擬、S-3 全案が走れた時間だけの KPI、S-4 区間別、S-5 走行状態別
  ##   S-6 ペダル応答 ±20% での頑健性
  ##   S-7 過去 2 列だけ実測にしたときの指令の変化（手順 3 の実走行ログ上で、偏差の符号ごとの向き）
  ##   S-8 アクセル予測開度が v0 のどこで最大になるか（C4 の効く向きが反転する理由）
  ##   S-9 指令の細かさ（ペダル切替の回数と開度の動き [%/s]。実機での振動の目安）
  ##   S-10 C5 に むだ時間 0.2s・車速の量子化・指令のレート制限を入れたときの壊れ方
  ##   候補: C0 現行 / C1 惰行カーブで選ぶ＋効く行だけ学習＋不感帯以上 / C2 C1＋ブレーキは物理式 /
  ##         C3 C1＋先読みを遅れ分ずらす / C4 C1＋動作点は実車速 /
  ##         C5 C1＋t 以前は実測・t 以降は基準の絶対値（FF の中にフィードバックが入る案）
  ##   図は results/reportYYYYMMDD_KAIZEN_process2,3/compare_*.png
  `.venv/bin/python -m tests.research.kaizen --part compare; echo "exit=$?"`

  # 段階 3 手順 2 パターン走行の網羅性と所要時間（PostgreSQL が必要。約 1 分。閉ループ模擬をしない）
  ##   V-1 速度帯 × 開度で「WLTP が要る時間 / 手順 2 の効いている行数」（太字が埋まっていないセル）
  ##   V-2 開度ごとの分布（速度をまとめたもの）、V-3 パターンごとの実績（所要時間・開度・開始車速）
  ##   V-4 フェーズごとの所要時間（打ち切りに張り付いていないか）、V-5 現行パターンの欠陥
  ##   V-6 パターン案と所要時間（実測の単位時間から積算。車両モデルでは模擬しない）
  ##   V-7 学習の系統を 1 つ抜いて学習し直したときの WLTP 指令開度の変化（パターン構成の効き）
  ##   V-8 実開度 PNOW（0x9000）を毎周期の CNOW 読み取りに相乗りさせる案のフレーム長
  ##   「要る開度」は基準車速から物理式（惰行カーブ＋ペダルゲイン）で逆算する（モデル予測は使わない）
  ##   図は results/reportYYYYMMDD_KAIZEN_process2,3/（coverage_map.png・coverage_openings.png）
  `.venv/bin/python -m tests.research.kaizen --part coverage; echo "exit=$?"`

  ## レポート: tests/research/results/report20260912_KAIZEN_process2,3.md
  ##   （段階 1 は 1・2 章、段階 2 は 3 章、段階 3 は 4 章）
## 手順 2・手順 3 のデバッグ（改善案 C1 の実装。2026-09-12）
`tests/research/results/report20260912_KAIZEN_process2,3.md` 5 章 表 5-5 の実装順 1〜2
（走行の要らない ◎ 項目 B1〜B3・A1）を実装したもの。**`src/` は変更していない**
（研究専用の `tests/research/ff_candidate.py` に実装し、手順 2・3 の呼び出し先だけ差し替えた）。

  `cd /home/raspi5_16gb/projects/driving_robot`

  # 変更のテスト（車両には触らない）
  `.venv/bin/python -m pytest tests/research/test_research_ff_candidate.py -q`

  # 走行せずに、既存の走行 CSV から FF モデルだけを作り直す（A1: 効いている行だけで学習）
  ##   --hw stub は config を更新しない（確認用）。--hw real で config_testVehicle.yaml へ書き戻す
  ##   期待値（レポート 表 3-2）: アクセル n=2291 MAE 1.30 / ブレーキ n=1686 MAE 0.52・
  ##   不感帯未満の予測はどちらも 0%
  `.venv/bin/python -m tests.research.main --retrain tests/research/results/drive_log_real_20260911_161247.csv --hw stub`
  `.venv/bin/python -m tests.research.main --retrain tests/research/results/drive_log_real_20260911_161247.csv --hw real`

  # スタブで手順 3 の流れだけ確認する（実機に触らない。先頭 90s で打ち切り）
  `.venv/bin/python -m tests.research.main --steps 1,3 --hw stub --limit-s 90`

  # 実機で手順 3 を走らせる（**アクチュエータが物理的に動く**。約 31 分）
  ##   実行順: 手順 1（初期化）→ 走行前チェック → 手順 3
  `.venv/bin/python -m tests.research.main --steps 1,3 --hw real`

  # 実機で手順 2 → 手順 3 を通す（パターン走行から作り直す場合。約 45 分）
  `.venv/bin/python -m tests.research.main --steps 1,2,3 --hw real`

## 手順3 実機走行のブレーキ脱落 → 安全網の追加（2026-09-13）

9/13 の手順3 実機走行（`drive_log_real_20260913_060556.csv`）で t=437.6s にブレーキ軸の電流が
脱落し、以降 778s（走行の 64%）を検知できないまま走っていたことが判明。`tests/research/mode_drive.py`
に **アラーム確認（1.0s ごと）** と **電流ゼロ継続（1.0s）** の安全網を追加した。`src/` は変更していない。

  `cd /home/raspi5_16gb/projects/driving_robot`

  # 変更のテスト（車両には触らない）
  `.venv/bin/python -m pytest tests/research/test_research_mode_drive.py tests/research/test_research_drive_log.py -q`

  # スタブで手順 3 の流れだけ確認する（安全網が誤検知しないことを含む。実機に触らない）
  `.venv/bin/python -m tests.research.main --steps 1,3 --hw stub --limit-s 90`

  # 手順 2 から測り直す（今日のハードで不感帯・停車保持開度・ペダルゲインを再実測。約 14 分）
  `.venv/bin/python -m tests.research.main --steps 1,2 --hw real`

  # 実機で手順 3 を再走行（**アクチュエータが物理的に動く**。安全網が入った状態。約 31 分）
  `.venv/bin/python -m tests.research.main --steps 1,3 --hw real`

  ##  CSV に governor_active（減速G ガバナー作動）・alarm_accel・alarm_brake の 3 列を追加した
  ##  （モード走行以外の区間・未確認の周期は空欄）。レポート 5 章に「ブレーキ寄与ほぼ0」の行を追加

## 手順3 急ブレーキ対策 1: ペダルの待機位置（2026-09-13）

使っていないペダルを 0% ではなく「不感帯 − 2%」（アクセル 8.00% / ブレーキ 11.16%）で待たせる。
`tests/research/config_testVehicle.yaml` の `mode_drive.pedal_standby`（false で従来通り 0%）・`standby_margin_pct`。

  `cd /home/raspi5_16gb/projects/driving_robot`

  # 変更のテスト（車両には触らない）
  `.venv/bin/python -m pytest tests/research/test_research_mode_drive.py tests/research/test_research_config.py -q`

  # スタブで手順 3 の流れだけ確認（起動時に「ペダルの待機位置: 8.00% / 11.16%」と出る。実機に触らない）
  `.venv/bin/python -m tests.research.main --steps 1,3 --hw stub --limit-s 90`

  # 実機で手順 3 を再走行（**アクチュエータが物理的に動く**。約 31 分）
  ##  手順2 は 9/13 07:15 の実測・モデルをそのまま使う。アラーム解除・原点復帰などハードを触ったら先に共有する
  `.venv/bin/python -m tests.research.main --steps 1,3 --hw real`

  ##  見るところ（9/13 07:27 と比べる）: ブレーキ切り替え時の最大減速（30〜62 → ≲10 km/h/s）、
  ##  基準 >5 km/h での完全停止（8 回 → 0）、Gガバナー作動（120.5s → ほぼ 0）、完走できるか、
  ##  アクセル中のブレーキ引きずり（加速中の平均偏差が悪化していないか）

## 手順2・3 デバッグのまとめ（2026-09-13）

`tests/research/results/report20260913_debag_process2,3.md` の表と図を、走行ログ CSV から作り直す（車両には触らない）。

  `cd /home/raspi5_16gb/projects/driving_robot`

  # 表（Markdown）をターミナルに出し、図を results/report20260913_debag_process2,3/ に保存する
  `.venv/bin/python -m tests.research.debug_process23`

  # 図の保存先を変える場合
  `.venv/bin/python -m tests.research.debug_process23 --out /tmp/debug_figs`

  # テスト
  `.venv/bin/python -m pytest tests/research/test_research_debug_process23.py -q`

## 手順2 A6: 打ち切りの見直し＋ガバナーの解除＋全パターン後の停車復帰（2026-09-13）

KAIZEN 表5-5 順3。`tests/research/pattern_loop.py` の変更（ガバナーの解除・停車復帰・打ち切り・安全網）。**`src/` は変更していない**。

  `cd /home/raspi5_16gb/projects/driving_robot`

  # 変更のテスト（車両には触らない）
  `.venv/bin/python -m pytest tests/research/test_research_pattern_loop.py tests/research/test_research_pattern_drive.py tests/research/test_research_mode_drive.py tests/research/test_research_debug_a6.py -q`

  # スタブで手順 2 を最後まで流す（実機に触らない。config_testVehicle.yaml は書き換えない。約 15 分）
  ##  見るところ: 運転パターンの後に「DRIVE_BRAKE」が入り、停車してから次のパターンへ進むこと・exit=0
  `.venv/bin/python -m tests.research.main --steps 1,2; echo "exit=$?"`

  # 実機で手順 2 を走らせる（**シャシダイナモ上で。車両が 0→約137 km/h を 17 回加減速する**。推定 15〜20 分）
  ##  実行順: 手順 1（初期化）→ 走行前チェック → ペダル探索 → パターン走行 → 緩減速・停車保持 → モデル作成
  ##  起動時に「各運転パターンの後は DRIVE_BRAKE で停車してから次へ進みます（60s 以内に停車しなければ中断）」
  ##  「Gガバナー: … × 0.7 未満で 0.5%/周期ずつ戻す」と出る。走行中の表示にガバナー作動中は「Gガバナー作動」と出る
  ##  中断（exit=5）したら理由が「PatternLoop が非常停止しました（…）」に出る（アラーム・電流 0mA・停車しない・過電流・CAN）
  ##  feedforward.model_path と物理定数は走行後に config_testVehicle.yaml へ保存される（前のモデル .pkl は残る）
  `.venv/bin/python -m tests.research.main --steps 1,2 --hw real; echo "exit=$?"`

  # 走行後: 前の手順 2（9/13 07:15）と比べる（車両には触らない。図は CSV と同じ名前の _a6 フォルダ）
  ##  関門: 運転パターンの開始車速が揃うか（全部停車から）・加速を終えた車速が cap 137.2 km/h 付近か・所要時間が 900s に収まるか
  `.venv/bin/python -m tests.research.debug_a6 --csv tests/research/results/drive_log_real_<日時>.csv`

## 手順2 A2・A5: 低開度の ACCEL_SWEEP と 60 km/h からの BRAKE_HOLD を追加（2026-09-13）

KAIZEN 表5-5 順4。本番のパターン列（28 本）に研究側で 6 本を足す（34 本）。**`src/` は変更していない**。

- 追加 ACCEL_SWEEP: 不感帯 + 2 / 5 / 8%（`learning.accel_sweep_add_offsets_pct`）
- 追加 BRAKE_HOLD: 60 km/h まで加速してから、不感帯 + 0.5 / 2.0 / 4.0% を保持（`learning.brake_hold_low_offsets_pct`・`brake_hold_low_start_kmh`）
- `learning.timeout_s` を 900 → 1200s（見積り約 873s。900s に収まるかはレポートで判定）

  `cd /home/raspi5_16gb/projects/driving_robot`

  # 変更のテスト（車両には触らない）
  `.venv/bin/python -m pytest tests/research/test_research_pattern_loop.py tests/research/test_research_pattern_drive.py tests/research/test_research_config.py tests/research/test_research_debug_a2a5.py -q`

  # スタブで手順 2 を最後まで流す（実機に触らない。config_testVehicle.yaml は書き換えない。約 20 分）
  ##  見るところ: パターン一覧が 34 本で、12〜14 が ACCEL_SWEEP（不感帯 + 2/5/8%）、27〜29 が「（60 km/h まで加速）」の BRAKE_HOLD・exit=0
  `.venv/bin/python -m tests.research.main --steps 1,2; echo "exit=$?"`

  # 実機で手順 2 を走らせる（**シャシダイナモ上で。車両が 0→約130 km/h を 17 回、0→60 km/h を 3 回、低開度（12〜18%）の加速を 3 回行う**。推定 15〜20 分）
  ##  パターン一覧の見出しが「本番 LearningDriveManager.generate_patterns ＋研究側の追加、34 本」になっていること
  ##  中断（exit=5）したら理由が「PatternLoop が非常停止しました（…）」に出る
  ##  feedforward.model_path と物理定数は走行後に config_testVehicle.yaml へ保存される（前のモデル .pkl は残る）
  `.venv/bin/python -m tests.research.main --steps 1,2 --hw real; echo "exit=$?"`

  # 走行後: A6 の手順 2（9/13 14:54）と比べる（車両には触らない。WLTP は DB から読む。図は CSV と同じ名前の _a2a5 フォルダ）
  ##  関門: 所要時間が 900s に収まるか・表 4-1 の空白（A6 はアクセル 12 個 / ブレーキ 4 個）が減るか・最高速が 140 km/h 以下か
  ##  あわせて: 追加 ACCEL_SWEEP の終わりの車速と一定になったか、60 km/h からの BRAKE_HOLD が保持中に停車したか
  `.venv/bin/python -m tests.research.debug_a2a5 --csv tests/research/results/drive_log_real_<日時>.csv`

## 手順2 A3・A4: トリム階段と 20 km/h からの高ブレーキ保持を追加（2026-09-13）

KAIZEN 表5-5 順5。A2・A5 のパターン列（34 本）に研究側で 7 本を足す（41 本）。**`src/` は変更していない**。

- A4: 不感帯 + 8%（18%）で 20 km/h まで加速してから、ブレーキ不感帯 + 7 / 17 / 27 / 37%（≈20/30/40/50%）を停車まで保持（30〜33）
- A3: 120 / 90 / 50 km/h まで 70% で加速してから、不感帯 + 8 → 5 → 2%（18 → 15 → 12%）を各 8s 保持 → 停車復帰（39〜41）。cap に達したら低い段へ下げる
- `learning.timeout_s` は 1200s のまま（見積り約 1020s。900s は記録だけ）

  `cd /home/raspi5_16gb/projects/driving_robot`

  # 変更のテスト（車両には触らない）
  `.venv/bin/python -m pytest tests/research/test_research_pattern_loop.py tests/research/test_research_pattern_drive.py tests/research/test_research_config.py tests/research/test_research_debug_a2a5.py tests/research/test_research_debug_a3a4.py -q`

  # スタブで手順 2 を最後まで流す（実機に触らない。config_testVehicle.yaml は書き換えない）
  ##  見るところ: パターン一覧が 41 本で、30〜33 が「（20 km/h まで加速）」の BRAKE_HOLD、39〜41 が「トリム階段 18.0 → 15.0 → 12.0% 各 8s」・exit=0
  `.venv/bin/python -m tests.research.main --steps 1,2; echo "exit=$?"`

  # 実機で手順 2 を走らせる（**シャシダイナモ上で。0→約130 km/h を 17 回、0→60 km/h を 3 回、低開度（12〜18%）の加速を 3 回に加え、0→20 km/h を 4 回（高ブレーキで停車）、0→120 / 90 / 50 km/h からのトリム階段を 1 回ずつ行う**。パターン走行は約 17 分の見込み）
  ##  パターン一覧の見出しが「本番 LearningDriveManager.generate_patterns ＋研究側の追加、41 本」になっていること
  ##  トリム階段で 140 km/h を超えたら最高速超えの回復（ブレーキ 30%）が働き、そのパターンは停車復帰して次へ進む
  ##  中断（exit=5）したら理由が「PatternLoop が非常停止しました（…）」に出る
  ##  feedforward.model_path と物理定数は走行後に config_testVehicle.yaml へ保存される（前のモデル .pkl は残る）
  `.venv/bin/python -m tests.research.main --steps 1,2 --hw real; echo "exit=$?"`

  # 走行後: A2・A5 の手順 2（9/13 17:27）と比べる（車両には触らない。WLTP は DB から読む。図は CSV と同じ名前の _a3a4 フォルダ）
  ##  関門: 表 4-1 の空白（A2・A5 はアクセル 8 個 / ブレーキ 2 個）が減るか・最高速が 140 km/h 以下か・所要時間が 1200s 以内か（900s との差は記録）
  ##  あわせて: 階段の各段を終えた理由（保持時間 / cap / 低速 / 最高速超え）、20 km/h からの保持で 0〜20 km/h × 40% 以上の行が採れたか
  `.venv/bin/python -m tests.research.debug_a3a4 --csv tests/research/results/drive_log_real_<日時>.csv`

## 手順2・3 A7: 実開度とステータスを記録し、ログの列を手順 1 から共通に（2026-09-13）

KAIZEN 表5-5 順6。両軸を毎周期 0x9000 から 14 レジスタまとめ読みし、指令（FF・最終指令）・実開度・ステータス・1 周期の処理時間を全区間で同じ列に残す。**`src/` は変更していない**。

  `cd /home/raspi5_16gb/projects/driving_robot`

  # 変更のテスト（車両には触らない）
  `.venv/bin/python -m pytest tests/research/test_research_hardware.py tests/research/test_research_drive_log.py tests/research/test_research_pattern_loop.py tests/research/test_research_mode_drive.py tests/research/test_research_debug_a7.py -q`

  # スタブで手順 3 を 60s だけ流す（実機に触らない）
  ##  見るところ: 「1 周期の処理時間: 平均 … / p95 … / 最大 … ms」が出て exit=0。CSV のヘッダーが accel_ff_pct・accel_cmd_pct・accel_actual_pct・accel_servo_on などになっている
  `.venv/bin/python -m tests.research.main --steps 1,3 --limit-s 60; echo "exit=$?"`

  # 実機で手順 2 を走らせる（A3・A4 の確認を兼ねる。**シャシダイナモ上で**。内容は上の「手順2 A3・A4」と同じ）
  `.venv/bin/python -m tests.research.main --steps 1,2 --hw real; echo "exit=$?"`

  # 実機で手順 3 を走らせる（**シャシダイナモ上で。WLTP 1800s**）
  ##  走行後の表示「1 周期の処理時間」で p95 が 50ms を超えていないか、「1 周期以上の遅れ」が増えていないかを見る
  `.venv/bin/python -m tests.research.main --steps 1,3 --hw real; echo "exit=$?"`

  # 走行後: A7 の確かめ（車両には触らない。図は CSV と同じ名前の _a7 フォルダ）
  ##  関門: 1 周期の処理時間の p95（パターン走行 ≤ 100ms・モード走行 ≤ 50ms）、実開度が読めなかった行、アラームコード
  ##  あわせて: 指令と実開度の差（踏み込み / 戻し / 保持）、指令が変わってから実開度が追いつくまでの遅れ
  `.venv/bin/python -m tests.research.debug_a7 --csv tests/research/results/drive_log_real_<日時>.csv`

## 手順2 A3・A4 ＋ A7 の実機結果の解析（2026-09-13 21:44 走行分）

  `cd /home/raspi5_16gb/projects/driving_robot`

  # A3・A4 の関門（前 = A2・A5 の 17:27 と比較。車両には触らない）
  `.venv/bin/python -m tests.research.debug_a3a4 --csv tests/research/results/drive_log_real_20260913_214423.csv`

  # A7 の関門（1 周期処理時間・実開度と指令の差・遅れ・ステータス。車両には触らない）
  `.venv/bin/python -m tests.research.debug_a7 --csv tests/research/results/drive_log_real_20260913_214423.csv`

  # 参考: レポート 4〜6 章の補足集計（読み取り専用の一時スクリプト。恒久コードではない）
  ##  一定開度での加速度（高速域の車両挙動）・パターン単位の分割検証 MAE・WLTP 開ループ比較・ラベルを実開度にした試算
  ##  スクリプトはスクラッチパッドに置いた一時ファイルなので tests/research には無い（再現したい場合は報告依頼のこと）

  レポート: `tests/research/results/report20260913_debag_process2,3_A3A4_A7.md`

## D1（学習ラベル）の準備: 実開度ラベルのモデルを作る（2026-09-14、車両には触らない）

  `cd /home/raspi5_16gb/projects/driving_robot`

  # 指令ラベル（従来通り。比較用に再生成。--out-dir を付けなければ tests/research/results/models/ に保存）
  `.venv/bin/python -m tests.research.train_candidate_model tests/research/results/drive_log_real_20260913_214423.csv --label cmd`

  # 実開度ラベル（PNOW 由来。D1 比較走行の 2 本目に使う）
  `.venv/bin/python -m tests.research.train_candidate_model tests/research/results/drive_log_real_20260913_214423.csv --label actual`

  # C3 用（先読み 0.5s ずらし。ラベルは D1 で決まった方に揃える。例は cmd ラベル）
  `.venv/bin/python -m tests.research.train_candidate_model tests/research/results/drive_log_real_20260913_214423.csv --label cmd --shift-lookahead 0.5`

  実開度ラベルの pkl: `tests/research/results/models/test_vehicle_20260913_203233.pkl`
  （パターン単位 GroupKFold CV MAE: アクセル 0.75%・ブレーキ 0.72%。指令ラベルの pkl は 1.17%/1.17%。
  `report20260913_debag_process2,3_A3A4_A7.md` 5.3 節の試算値と一致）

  テスト: `.venv/bin/python -m pytest tests/research/test_research_train_candidate_model.py tests/research/test_research_drive_log.py tests/research/test_research_model_analysis.py -q`

## D1 の実機比較走行（次にユーザーが実機で行う。手順3・candidate は C1 のまま）

  # 先に D3: A7 のモード走行 50ms 周期の確認（短時間）
  `.venv/bin/python -m tests.research.main --steps 1,3 --hw real --limit-s 120; echo "exit=$?"`

  # config_testVehicle.yaml の feedforward.model_path を指令ラベル pkl にして 1 本目
  `.venv/bin/python -m tests.research.main --steps 1,3 --hw real; echo "exit=$?"`

  # feedforward.model_path を実開度ラベル pkl（test_vehicle_20260913_203233.pkl）に替えて 2 本目
  `.venv/bin/python -m tests.research.main --steps 1,3 --hw real; echo "exit=$?"`

  2 本走らせたら CSV パスを Claude に伝える（compare_runs.py で追従性を比較する）。

## D1 の比較（2 本走らせたあと。車両には触らない）

  `.venv/bin/python -m tests.research.compare_runs <CSV1> <CSV2> --labels "指令ラベル" "実開度ラベル"`

  勝った方の model_path を C2・C3・C4・C5 用の既定にする。

## V1〜V3 実装後のテスト（2026-09-14、車両には触らない）

  `cd /home/raspi5_16gb/projects/driving_robot`
  `.venv/bin/ruff check tests/research`
  `.venv/bin/python -m pytest tests/research -q`

  # スタブで C1〜C5 の候補スイッチを確認（config の feedforward.candidate を差し替えて実行）
  `.venv/bin/python -m tests.research.main --steps 1,3 --limit-s 10 --config <candidate ごとに書き換えた一時 config>`

## D2 修正 → 手順2 やり直し（2026-09-14、上の「D1（学習ラベル）の準備」「D1 の実機比較走行」は車両特性修正前のCSVを使っており無効）

  # 実機で手順2をやり直す（シャシダイナモ上。ユーザー作業）
  `.venv/bin/python -m tests.research.main --steps 1,2 --hw real; echo "exit=$?"`

  # 新CSVでA3・A4／A7の関門を再確認（車両には触らない）
  `.venv/bin/python -m tests.research.debug_a3a4 --csv tests/research/results/drive_log_real_<新日時>.csv`
  `.venv/bin/python -m tests.research.debug_a7 --csv tests/research/results/drive_log_real_<新日時>.csv`

  # 新CSVから指令ラベル・実開度ラベルの2モデルを作り直す（車両には触らない）
  `.venv/bin/python -m tests.research.train_candidate_model tests/research/results/drive_log_real_<新日時>.csv --label cmd`
  `.venv/bin/python -m tests.research.train_candidate_model tests/research/results/drive_log_real_<新日時>.csv --label actual`

  # 以降は「D1 の実機比較走行」と同じ手順を、新しい2つのpklで実施する

## D2 修正後の結果（2026-09-14、`drive_log_real_20260914_064855.csv`）

  # 実施済み・結果は docs/memo.md 参照
  指令ラベル: tests/research/results/models/test_vehicle_20260913_220759.pkl
  実開度ラベル: tests/research/results/models/test_vehicle_20260913_220801.pkl

  # 次: D1の実機比較（--steps 1,3 --hw real を2本、feedforward.model_path を上の2つに差し替えて実施）
  # 走行後、以下でランキング判定
  `.venv/bin/python -m tests.research.compare_runs <D1比較で得られた2本のCSV>`

## 定速階段＋C6（2026-09-14 計画。段2・段3 の実装後に使う）

  # 段2 実装後: スタブで手順2（CRUISE_HOLD の行が各車速ぶん出るか・所要時間）
  `.venv/bin/python -m tests.research.main --steps 1,2; echo "exit=$?"`

  # 段4-1: 実機で手順2をやり直す（ユーザー作業）
  `.venv/bin/python -m tests.research.main --steps 1,2 --hw real; echo "exit=$?"`

  # 段4-2: 関門（車両には触らない）。2026-09-15 実施済み: drive_log_real_20260915_052432.csv（全関門○・11車速すべて保持。結果は docs/memo.md）
  # 注意: 手順2は vehicle.max_speed_kmh 140、手順3の比較走行は 180 に切り替える（180 のまま手順2を走ると 160 km/h 張り付きで所要時間が延びる）
  `.venv/bin/python -m tests.research.debug_a3a4 --csv tests/research/results/drive_log_real_<新日時>.csv`
  `.venv/bin/python -m tests.research.debug_a7 --csv tests/research/results/drive_log_real_<新日時>.csv`
  `.venv/bin/python -m tests.research.cruise_curve tests/research/results/drive_log_real_<新日時>.csv`
  # ↑ cruise_curve.py は段B（骨格＝実測テーブル、2026-09-15 変更）で実装済み。052432 で 11 段すべて保持

  # 段4-3: 新 CSV から C1 / C6 のモデルを作る
  `.venv/bin/python -m tests.research.train_candidate_model tests/research/results/drive_log_real_<新日時>.csv --label actual`
  `.venv/bin/python -m tests.research.train_candidate_model tests/research/results/drive_log_real_<新日時>.csv --label actual --cruise-curve-from tests/research/results/drive_log_real_<新日時>.csv`

  # ↑ 段C（2026-09-15 実装済み）。C6 の pkl には cruise_curve キーが入る。C1 の pkl を C6 で読むと ValueError で止まる
  #   分割検証は全体に加えて車速帯（0-40/40-80/80-120/120- km/h）別にも出る（参考値。採否は実機比較）

  # 段4-4 の前にスタブで C6 が回るか（config は触らず、コピーに candidate: C6 と model_path を書いて --config で渡す）
  `.venv/bin/python -m tests.research.main --steps 1,3 --hw stub --limit-s 10 --config <C6用configのコピー>; echo "exit=$?"`

  # 段4-4: 手順3を 2 本（feedforward に candidate: C1 / C6 を書き、model_path をそれぞれの pkl、vehicle.max_speed_kmh: 180）
  `.venv/bin/python -m tests.research.main --steps 1,3 --hw real; echo "exit=$?"`
  `.venv/bin/python -m tests.research.compare_runs <C1のCSV> <C6のCSV> --labels "C1 実開度" "C6 実測テーブル骨格"`

  # 段4-4 実施済み（2026-09-15）: drive_log_real_20260915_073019.csv（C1）/ drive_log_real_20260915_081715.csv（C6）
  # → C1 採用（C6 不採用）。分析・ユーザー決定は docs/memo.md 参照

## C1×3 → C5×3 → C4×3（2026-09-15。C1 採用後の次の比較。C2/C3 は今回対象外）

  # config は共通（feedforward.candidate だけ C1 → C5 → C4 と書き換えて 3 本ずつ走る）
  #   vehicle.max_speed_kmh: 180
  #   feedforward.model_path: tests/research/results/models/test_vehicle_20260914_221700.pkl（3 候補とも不変）
  #   feedforward.candidate: C1 / C5 / C4（このキーだけ変える）

  # 各候補で 3 本（実機。ユーザー作業）
  `.venv/bin/python -m tests.research.main --steps 1,3 --hw real; echo "exit=$?"`

  # 9 本まとめてランキング（--labels は省略。candidate 列からラベルが付く）
  `.venv/bin/python -m tests.research.compare_runs <C1の3本CSV> <C5の3本CSV> <C4の3本CSV>`

  # 注意: C2 は今回走らない（ブレーキ上限を下げる安全策を入れてから走る。値は未定）。C3 も対象外（前提の 0.5s 遅れが手順3 に無い）
