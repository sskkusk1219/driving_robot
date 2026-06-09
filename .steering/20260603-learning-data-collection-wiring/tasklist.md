# タスクリスト: 走行ログ収集・永続化の配線（学習運転データ収集）

> **状態: 実装中（2026-06-04 着手）。** 案A（連続走行ログ）＋学習用基準速度プロファイルのプロファイル自動生成で確定。

## 🚨 タスク完全完了の原則
着手時は全タスクを `[x]` にするまで継続。スキップは技術的理由のみ明記。

---

## フェーズ0: 設計決定
- [x] 学習データ方針を決定（**案A: 連続走行ログ DriveLog** に確定。train_inverse_model が連続ログのみを消費するため）
- [x] 案A採用時の「学習用基準速度プロファイル」の内容を定義（**VehicleProfile から自動生成**: max_speed/max_decel_g を網羅する加減速トレンドを `LearningDriveManager.build_learning_reference` でサーバ生成。学習画面はモード選択不要）

## フェーズ1: LogWriter 配線
- [x] `app.state.db_pool` から `LogWriter` を生成する経路を用意（in-memory 時はログ無効）
- [x] factory / 起動経路に LogWriter を組み込む（`LogWriter` が `asyncpg.Pool` を受け取れるよう型を拡張。Pool.execute が接続を都度取得・解放するため Web 駆動のセッション寿命に適合。`get_log_writer` 依存を追加）

## フェーズ2: 自動走行ログ記録
- [x] Web `/start` で mode/profile/log_writer を解決し `start_auto_drive` に渡す
- [x] `DriveLoop` 起動でログが `drive_logs` に記録されることを確認（mode/profile/calibration/ff/safety_check 揃い時に起動・log_writer 経路で記録）
- [x] セッション終了時に `end_session` を呼ぶ（`_open_session`/`_close_session` でセッション採番と終了を一元化。session_id は DB 採番値を使い FK 整合を確保）

## フェーズ3: 学習運転オーケストレーション
- [x] `start_learning_drive` を実データ収集フローに実装
  - [x] `LogWriter.start_session(run_type='learning')` で session 作成（`_open_session` 経由）
  - [x] DriveLoop を起動して学習用基準プロファイルを走行（`_active_learning_task` ではなく既存 DriveLoop の call_later スケジューリングを流用。案A で run_pattern 非依存のため専用タスクは不要）
  - [x] 100ms 周期で `DriveLog` を保存（DriveLoop の log_writer 経路）
  - [x] 完了で RUNNING→READY、`end_session('completed')`（on_complete=stop_auto_drive → `_close_session('completed')`）
- [x] `stop` / emergency でタスク cancel・session 終了・原点復帰（`stop`/`stop_auto_drive`→`_close_session('completed')`、`emergency_stop`→`_close_session('emergency')`。DriveLoop.stop() で記録中断）
- [x] ~~（案B時）`LearningLog`→`DriveLog` 変換層~~（案A 採用により不要）
- [x] 初回学習走行（モデル未ロード）対応: `FeedforwardController.has_model` を追加し、DriveLoop は未ロード時 FF=0・PID のみで追従（収集ログから初回モデルを学習するブートストラップ）
- [x] `LearningDriveManager.build_learning_reference(profile)` で学習用基準速度プロファイルを自動生成

## フェーズ4: 依存注入
- [x] factory / stubs で必要依存（learning_manager・LogWriter）を注入（factory/stubs に `LearningDriveManager` を注入。LogWriter は Web 層 `get_log_writer` 依存で `app.state.db_pool` から生成）

## フェーズ5: テスト
- [x] `test_robot_controller`: 学習運転のセッション作成・ログ書込・状態遷移（`TestDriveSessionLogging`: auto/learning の start_session 採番・DriveLoop 起動・stop/emergency の end_session 記録・log_writer なし時の UUID 採番）
- [x] `test_web_drive`: `/start` の mode/profile/log_writer 解決・`/learning/start` の log_writer 受け渡し・`get_log_writer` の有無分岐
- [x] 追加: `test_learning_drive`（build_learning_reference）/ `test_feedforward`（has_model）/ `test_drive_loop`（モデル未ロード時 PID のみ）
- [x] ~~integration（任意）: 実 DB でのセッション＋ログ保存~~（既存 `tests/integration/test_log_writer_db.py` が LogWriter↔実 DB を担保。本配線の実 DB 確認はフェーズ7 で実施。任意のため新規追加は見送り）

## フェーズ6: 品質チェック
- [x] `.venv/bin/pytest tests/unit/`（431 passed）
- [x] `.venv/bin/ruff check` / `ruff format --check`（変更分クリーン。残る E501 1件は本作業前からの既存 uncommitted 差分で本変更分外）
- [x] `.venv/bin/mypy`（変更モジュール 9ファイル: no issues）

## フェーズ7: 実機確認（要実機・PostgreSQL。本開発セッションでは実行不可）
> 実アクチュエータ/CAN/UPS/GPIO と PostgreSQL を要するため、デバイス上でユーザーが実行する。手順:
> 1. `DATABASE_URL` を設定し `python scripts/setup_db.py` でテーブル作成
> 2. `DRIVING_ROBOT_USE_REAL_HW=1` で起動し、プロファイル選択→キャリブレーション
> 3. 学習運転を開始→基準プロファイル走行→正常終了（RUNNING→READY）で自動学習が成功し `model_path`/`feedforward_params` が更新されることを確認
> 4. `drive_sessions`(run_type='learning') 1件・`drive_logs` が周期的に蓄積されていることを SQL で確認
> 5. 自動走行でログが蓄積され、再学習に使えることを確認
- [ ] 学習運転 → 終了 → 自動学習成功 → `model_path`/`feedforward_params` 紐付けを確認（**要実機: ユーザー実行**）
- [ ] 自動走行でログが蓄積され、再学習に使えることを確認（**要実機: ユーザー実行**）

## フェーズ8: 振り返り
- [x] 実装後の振り返りを記録

---

## 関連実装（完了済み・本作業の前提）
- `.steering/20260531-ridge-inverse-model/` — 連続ログからの先読み Ridge 逆モデル学習 + `/learning/train`
- `.steering/20260603-feedforward-dynamics-params/` — FF 物理定数（クリープ・不感帯ほか）+ 学習時自動推定
- `screens/learning.js` — 学習運転終了時の自動学習トリガー（手動ボタン廃止済み）

## 実装後の振り返り

**実装完了日**: 2026-06-04（フェーズ7 の実機確認を除き完了）

**計画と実績の差分**:
- フェーズ3 で当初想定した `_active_learning_task`（asyncio.Task で run_pattern を実行）は採用せず、既存 `DriveLoop`（call_later スケジューリング）を流用した。案A（連続走行ログ）では学習走行も自動走行と同じ「基準速度を追従して連続ログを記録する」処理であり、別タスク機構を作る必要がなかったため。`_active_learning_task` フィールドは `shutdown()` の既存テスト互換のため残置。
- セッション ID の採番方針を変更。当初は RobotController が UUID を採番していたが、`drive_logs.session_id` が `drive_sessions(id)` への FK のため、**DB 採番（`LogWriter.start_session` の返り値）を session_id として一貫使用**する `_open_session`/`_close_session` ヘルパーを新設し、auto/learning/manual の終了処理を一元化した。
- `LogWriter` は `asyncpg.Pool` を受け取れるよう型を拡張（Pool.execute が接続を都度取得・解放するため、Web 駆動で寿命が不定な走行セッションに適合）。新クラスを作らず最小変更で対応。
- 初回学習走行（モデル未ロード）への対応が必須と判明。`FeedforwardController.predict` は未ロード時に RuntimeError を送出するため、`has_model` プロパティを追加し、DriveLoop は未ロード時に FF=0・PID のみで追従するブートストラップとした。

**学んだこと**:
- 調査時に「部品はあるがオーケストレーションが無い」状態を正確に把握できたことで、DriveLoop/LogWriter の再利用に倒せた。新規機構を足すより既存の連続ログ経路へ寄せる判断が効いた。
- FK 制約（session → profile, log → session）を先に確認したことで、ID 採番の不整合（コントローラ採番と LogWriter 採番の二重化）という潜在バグを実装前に解消できた。
- 着手時に修正前から存在した mypy エラー（CalibrationManagerProtocol.save_manual 欠落）と ruff E501 を切り分け、本変更分のクリーン性を担保した。save_manual はプロトコルへ追記して解消。

**次回への改善提案**:
- 学習用基準速度プロファイルの内容（網羅レンジ・加減速レート）は実機データの質を見て調整余地あり。`LEARNING_*_RATE_FRACTIONS` / `LEARNING_DWELL_S` を将来プロファイル設定化する検討。
- 実機確認（フェーズ7）は本セッションでは実行不可。デバイス上での収集→学習→紐付けの実測が次アクション。
