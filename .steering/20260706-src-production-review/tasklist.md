# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 「実装が複雑すぎるため後回し」は禁止
- 未完了タスク(`[ ]`)を残したまま作業を終了しない

### タスクスキップが許可される唯一のケース
技術的理由(実装方針変更・アーキテクチャ変更・依存関係変更)のみ。スキップ時は理由を明記:
```markdown
- [x] ~~タスク名~~（実装方針変更により不要: 具体的な技術的理由）
```

---

> **出典**: /code-review high(8角度スキャン+検証)による src 全体レビュー(2026-07-06)。
> 各項目の検証判定: ✅=CONFIRMED, ⚠️=PLAUSIBLE。修正方針の詳細は design.md 参照。
> **実装は Sonnet が担当。フェーズ1→2→3→4→5 の順に実施すること(フェーズ3の共通化はフェーズ1・2の修正を取り込んで行う)。**

## フェーズ1: 安全性クリティカル修正(9件)

- [x] **D1** ✅ `src/domain/control/learning_loop.py:515` — COAST_DOWN パターン中のオーバースピード回復が DRIVE_BRAKE に入っても `pattern.brake_opening=0.0` のためブレーキ0%のまま、タイムアウト後に70%アクセルで再加速する
  - [x] 回復用 DRIVE_BRAKE では実効ブレーキ開度(>0)を用いる修正
  - [x] 回帰テスト: COAST_DOWN 中の overspeed → ブレーキ開度 > 0 を検証

- [x] **D3** ✅ `src/domain/control/schedule_loop.py:259` — ScheduleLoop のみ `enforce_pedal_exclusion` を通らず、タイムラインの重複区間で物理的にアクセル・ブレーキ同時踏みが発生する(pedal_safety.py の機構保護不変条件に違反)
  - [x] TimeSchedule 保存時バリデーションで同時非ゼロ区間を reject(schemas.py)
  - [x] 実行時にも enforce_pedal_exclusion を適用(既存データへの防御)
  - [x] ※同時踏みを仕様として許可したい場合の判断はユーザーへ確認事項として振り返りに記録

- [x] **D6** ✅ `src/domain/control/drive_loop.py:409` + `src/domain/control/pedal_arbiter.py:66,143` — arbitrate に渡す dt が無クランプのため、~0.9s のバスストール復帰時に rate_limit が 180-270% の1サイクルジャンプを許し、ペダルがアクチュエータ最高速で急動作する
  - [x] pedal_arbiter.arbitrate 入口で dt を公称 tick の 0.5×〜4× にクランプ(pid.py:63 と同方式)
  - [x] 回帰テスト: dt=0.95s でも 1tick 分のステップ上限を超えない

- [x] **D2** ⚠️ `src/domain/calibration.py:165-189` — _probe_contact のベースラインが接触後電流で汚染されるとスパイク検出が発火せず、+50パルス/50ms で最大50,000パルスまでハードストップに押し込む(過電流中断なし)
  - [x] ベースライン取得時の接触済み検出(閾値超なら CalibrationDetectionError)
  - [x] 探索ループに絶対電流上限による中断を追加

- [x] **I1** ✅ `src/infra/button_servo_driver.py:191` — press() に try/finally がなく、キャンセルや I2C エラーでサーボがボタン(エンジンスタート等)を押したまま保持される
  - [x] try/finally 化し finally で rest_angle 復帰をリトライ(失敗時ログ+servo off)

- [x] **I4** ✅ `src/infra/ups_monitor.py:216` — AC喪失コールバックの run_coroutine_threadsafe future が破棄され例外が無音で消える(しかもループスレッド内からの誤用)
  - [x] asyncio.create_task + 失敗ログ用 done-callback に変更(gpio_monitor.py:147 と同パターン)

- [x] **W2** ✅ `src/app/robot_controller.py:668` — stop() が `_drive_complete` を set しないため PID チューニング走行の停止が total_duration+30s までブロックし、タイムアウト後の release_stop_hold/home_return が新しい走行の Modbus コマンドと干渉し得る
  - [x] stop() で _drive_complete も set する
  - [x] release_stop_hold を呼ぶ except パスで現在状態を確認してから home_return

- [x] **W3** ✅ `src/app/robot_controller.py:653,1138` — stop()/stop_auto_drive() が home_return/servo_off の完了前に READY へ遷移するため、直後の arm が受理され、遅延実行の home_return がブレーキ保持を解除し得る(コントローラ変異を直列化するロックも無い)
  - [x] home_return/servo_off 完了後に READY へ遷移する順序に変更
  - [x] 統合テスト: stop→即arm で brake hold が維持される

- [x] **W6** ✅ `src/app/robot_controller.py:808-809` — run_calibration の finally が無条件 `_transition(READY)` のため、キャリブレーション中の EMERGENCY 後に InvalidStateTransition が元の結果/例外をマスクする。emergency_stop はキャリブレーション動作を停止させないため Modbus 書込が干渉する
  - [x] finally は現在状態が CALIBRATING の場合のみ READY へ遷移
  - [x] CalibrationManager に cancel フックを追加し emergency_stop から停止させる

## フェーズ2: 正確性修正(8件)

- [x] **W4** ✅ `src/app/robot_controller.py:1081-1088,1159-1160` — profile/calibration/ff_controller/safety_check が None のときループビルダーが silent return し、RUNNING へ遷移済み+セッション開設済みのまま永遠に完了しないファントム状態になる
  - [x] None チェックを状態遷移前に移動、欠落時は InvalidStateTransition を送出
  - [x] ループビルダーの silent return を例外送出に変更

- [x] **W5** ✅ `src/app/learning_cycle.py:158-165,186-187` — start() が arm 成功前に progress を ARMING/LEARNING に設定し失敗時に戻さないため、pre-check 失敗後も「実行中」が WS で永久配信される。アーミング中(_task=None)の abort() が 409 になる
  - [x] start() を try/except 化し失敗時に progress をリセット
  - [x] アーミング中の abort() をアーム中断(brake hold 解除+リセット)として処理

- [x] **W1** ✅ `src/web/routers/drive.py:608` — POST /drive/select-profile が InvalidStateTransition を catch せず、STANDBY/READY 以外で 500 を返す
  - [x] フェーズ3の **S4**(app レベル例外ハンドラ)で構造的に解消。S4 実施後にこのエンドポイントで 409 が返ることを確認 — test_select_profile_returns_409_on_invalid_state で確認済み

- [x] **I3** ✅ `src/infra/archive_manager.py:153-160,93,125` — セッション削除の2つの DELETE が非トランザクションで、中断後の再アーカイブが正常なアーカイブファイルをヘッダのみの CSV で上書きし恒久的にデータ喪失する
  - [x] 2つの DELETE を単一トランザクション化
  - [x] 0行時はアーカイブファイルを書かない(上書きしない)ガードを追加

- [x] **I5** ✅ `src/infra/mode_repository.py:93-107`, `src/infra/profile_repository.py:208-237` — update が UniqueViolationError を DuplicateNameError に変換せず、既存名へのリネームが 409 でなく 500 になる(create は変換済み)
  - [x] 両 update に create と同じ変換を追加

- [x] **I6** ⚠️ `src/infra/can_reader.py:73-90` — connect() が Bus オープン後に DBC 読み込み例外を送出し、Bus ハンドルがクローズされずリークする(single_handle チャネルが占有されたまま)
  - [x] DBC 読み込みを Bus オープン前に移動(または except で bus.shutdown() して re-raise)

- [x] **D4** ✅ `src/domain/pid_tuning.py:320` — tuning_cost が逆転回数を KPI 上限(1回/5s)でなく窓長 5.0s で割っており、振動ペナルティが5倍過小 → チューナーが振動的ゲインを採用し得る
  - [x] KPI 上限値による正規化に修正
  - [x] コスト重みが変わるため既存プロファイルの再チューニング推奨を振り返りに記録

- [x] **D5** ✅ `src/domain/pid_tuning.py:141`, `src/domain/model_training.py:490-491` — deadband_pct=0.0(合法値)のとき strict `<` が常に False となり、FOPDT 同定と物理定数推定が無音で全滅する(falsy-zero)
  - [x] `<` を `<=` に変更(両ファイル)
  - [x] schemas.py の deadband フィールドに ge=0 制約を追加
  - [x] 回帰テスト: deadband=0.0 でセグメント検出が機能する

## フェーズ3: 構造改善(12件)

- [x] **S1** ✅ 3制御ループ(`drive_loop.py`/`learning_loop.py`/`schedule_loop.py`)が各~110-150行のループ基盤(_schedule_next_cycle ウォッチドッグ・_on_cycle_done・stop_and_join の shield/self-join 回避・_abort_emergency・ログバックログ管理・MAX_PENDING_LOG_TASKS)をコピペ共有しており、修正が1ループにしか適用されない(ストール計測は DriveLoop のみに存在)
  - [x] `src/domain/control/base_loop.py` に CycleLoopBase を新設(drive_loop.py の実装を正とする)
  - [x] 3ループを継承に書き換え(サイクル本体のみ各ループに残す)
  - [x] ストール計測を全ループで有効化
  - [x] 既存テスト全パスで挙動不変を担保

- [x] **A1** ✅ 開度→パルス変換 `_opening_to_position` が3ループ(schedule_loop.py:365, learning_loop.py:684, drive_loop.py:588)+RobotController(robot_controller.py:1487-1489,1503-1505 のインライン)に重複し、クランプ `_clamp_accel/_clamp_brake` も schedule_loop.py:359/learning_loop.py:626/learning_drive.py:113-144 に重複
  - [x] `src/domain/control/conversions.py` に opening_to_position / clamp_opening を新設し全箇所を置換
  - [x] RobotController の _decelerate_to_stop/_apply_brake_hold も同関数経由に変更

- [x] **A2** ✅ 停止判定閾値 0.5km/h が4箇所(robot_controller.py:44, learning_loop.py:35, pre_check.py:14, model_training.py:66)、G_TO_KMHS=9.81*3.6 が3箇所(learning_loop.py:36, robot_controller.py:52, pid_tuning.py:249)で独立定義され「同一であること」がコメント頼み
  - [x] conversions.py(または settings)の共通定数に集約し全参照を置換

- [x] **A3** ✅ `src/app/robot_controller.py:915-918` — _run_tuning_drive が PreCheckRunner をスキップしつつ READY→PRE_CHECK→RUNNING の空遷移で状態機械を形式的に満たしている(呼び出し側バイパス)
  - [x] 状態機械側に「ブレーキ保持中の再走行は READY→RUNNING を許可」等の第一級遷移(またはエントリアクション)として定義し直し、空遷移を排除

- [x] **A4** ✅ `src/app/robot_controller.py:1011,1019,1045,386-396` — 稼働中 PID ゲインの所有者が2つ(ライブ PIDController vs 永続 VehicleProfile)で、チューニング後に persist+refresh が失敗/スキップされると DB/UI 表示と実際の走行ゲインが乖離する
  - [x] ゲインは常にアクティブプロファイル経由(_apply_profile_to_control_stack)で適用する単一所有権に変更し、run_pid_tuning_session は結果を返すだけにする(呼び出し元で persist→refresh を必須化)

- [x] **S2** ✅ `src/app/robot_controller.py` — `gather(accel.home_return(), brake.home_return())` ブロックが11箇所(625,658,704,759,968,1141,1210,1259,1321,1426,1625)にコピペされ、release_stop_hold(960-970)が既に同内容
  - [x] `_home_both()` 私有ヘルパー(または release_stop_hold)に集約(※W3 の順序修正後の形で)

- [x] **S4** ✅ `src/web/routers/drive.py` — 27箇所の同一 try/except(InvalidStateTransition→409 / PreCheckFailed→422 / PidTuningAborted→409)が全エンドポイントにコピペされ、漏れたエンドポイント(W1)が 500 を返す
  - [x] web/app.py に app レベル exception handler を登録し、各エンドポイントの重複 try/except を削除((InvalidStateTransition, ValueError)→409 の箇所はローカル ValueError catch のみ残す)
  - [x] 他ルーター(modes.py 等)の同パターンも同時に整理

- [x] **S5** ✅ `src/web/routers/drive.py:69-78,457-469,612-623` — レスポンススキーマをフィールド手書きコピーで構築(11フィールドの FeedforwardParamsSchema ブロック等、~60行)
  - [x] スキーマに `model_config = ConfigDict(from_attributes=True)` を追加し `Schema.model_validate(obj)` に置換

- [x] **S7** ✅ `src/infra/profile_repository.py:27-63`, `src/web/routers/profiles.py:35-49` — dataclass⇔JSON/スキーマ変換が6組み近似重複(_ffp_*/_dyn_* 各種)
  - [x] 汎用ヘルパー(from-jsonb 用と attrs-copy 用の2フレーバー)に集約

- [x] **S8** ✅ `src/infra/profile_repository.py:136-144,197-205` — pid_json/stop_json の json.dumps ブロックが create/update で文字単位同一
  - [x] モジュールレベルの _pid_to_json/_stop_to_json に集約

- [x] **S3** ✅ `src/app/robot_controller.py:457-459` — _active_control_loop() の全身が `return self._realtime_loop`(呼び出し2箇所)
  - [x] メソッドを削除し直接参照に置換

- [x] **S6** ✅ `src/domain/control/learning_loop.py:169,198,573` — _stable_since は None 代入3箇所のみで一度も読まれない死んだ状態
  - [x] フィールドと代入を削除

## フェーズ4: 効率改善(7件)

- [x] **E1** ✅ `src/app/robot_controller.py:494-511` + `src/web/ws.py:78-87` — アイドル時(STANDBY/READY)も WS 10Hz が get_realtime_data のハードウェア読みフォールバック(CAN×1+Modbus×4/100ms)を叩き、ジョグ/キャリブレーション操作がモニタ読みの後ろに並ぶ
  - [x] アイドルスナップショットに短TTLキャッシュ(0.5-1s)を導入(またはアイドル時 1-2Hz にポーリング低減)

- [x] **E2** ✅ `src/domain/control/schedule_loop.py:66-77` — interpolate_pedal が毎tick先頭からO(n)線形走査(DriveLoop._ref_speed_at は同用途を precompute+bisect 済み)
  - [x] __init__ で時刻リストを precompute し bisect_right(または単調カーソル)に変更

- [x] **E3** ✅ `src/domain/control/schedule_loop.py:277-279,327-329` — 補間位置が不変でも毎100msに両軸へ move_to_position 書込(LearningLoop は最終指令位置を追跡して省略済み)
  - [x] 最終指令位置を記録し同値なら書込スキップ(learning_loop.py:643-672 と同方式)

- [x] **E4** ✅ `src/web/ws.py:76-145` — アイドルでも毎100msにフル RealtimeData 再構築+model_dump_json+全クライアント送信(タイムスタンプ以外不変)
  - [x] タイムスタンプ除外の変更検知で送信スキップ(またはアイドル時 1Hz にスロットル)

- [x] **E5** ✅ `src/infra/log_writer.py:96-114` — drive_log が1サンプル=1 INSERT=1タスク=1プール接続取得(10件/s、10時間で36万往復)。DB遅延時に100タスク上限でサンプルが無音ドロップ
  - [x] メモリバッファ+0.5-1s間隔の executemany(または copy_records_to_table)バッチフラッシュに変更、end_session で最終フラッシュ

- [x] **E6** ✅ `src/infra/ups_monitor.py:141-142,180-197` — 5秒毎のポーリングで NUT デーモンへの TCP 接続を2回張って2回 LOGOUT(変数ごとに接続)
  - [x] 1接続で両変数を取得(または持続接続+エラー時再接続)

- [x] **I2** ⚠️ `src/infra/archive_manager.py:95-132` — _export_to_csv/_compress/disk_usage がイベントループ上で同期実行され、大量アーカイブ時に WS/HTTP/緊急停止コルーチンを数秒〜数分凍結させる(※検証で check_and_archive が本番未配線と判明 — 現状は潜在バグ)
  - [x] ブロッキング I/O を asyncio.to_thread にオフロード
  - [x] check_and_archive の本番配線は仕様判断が必要なため、未配線である事実をユーザーへ報告(振り返りに記録) — 下記の振り返りセクション参照

## フェーズ5: 規約準拠と最終検証

- [x] **C1** ✅ development-guidelines.md の「フォーマッタ: `ruff format`(Black互換)」違反 — 8ファイルが `ruff format --check` 不合格: robot_controller.py, web/schemas.py, web/routers/drive.py, domain/pid_tuning.py, infra/settings.py, web/app.py, app/stubs.py, web/deps.py
  - [x] `ruff format src/ tests/` を実行(※フェーズ1-4の修正がすべて完了した後に実施し、機能修正とフォーマット変更のコミットを分けること) — フェーズ1-4実施中に編集した他ファイルも合わせてドリフトしていたため、最終的に28ファイルを整形(元の8ファイル含む)

- [x] 最終検証
  - [x] `pytest` 全件パス — 989 passed, 0 failed/error（統合テスト含む）
  - [x] `ruff check src/ tests/` エラーなし — All checks passed!
  - [x] `ruff format --check src/ tests/` パス — 112 files already formatted
  - [x] フェーズ1の各修正に回帰テストが追加されていることを確認 — 完了(前セッションで確認済み、S1-S4/A3/A4/E1-E6/I2 も本セッションで追加)

---

## 実装後の振り返り

- **実装完了日**: 2026-07-06
- **計画と実績の差分**:
  - 全37件を計画どおりフェーズ1〜5の順で実装。スキップした項目なし。
  - S1(CycleLoopBase 抽出)の過程で、副次的に LearningLoop の未使用フィールド
    `_complete_task` を発見・削除(dead code、テスト・grep で未参照を確認済み)。
  - S4(app レベル例外ハンドラ)の実施により、profiles.py/modes.py の PUT/PATCH
    エンドポイントに元々 DuplicateNameError ハンドリングが無かった穴も同時に解消。
  - S1 の共通化に伴い、ログメッセージの一部表記を3ループで統一(「緊急停止」→「非常停止」、
    ログ滞留メッセージへのループ種別ラベル付与)。動作・状態遷移は無変更、ログ文言のみの
    整理であることをテスト(caplog 依存なし)で担保。
- **学んだこと**:
  - PID 自動適合(run_pid_tuning_session/run_pid_validation)はライブ PID の一時的な
    ゲイン書き換えを行うため、探索終了後に必ずアクティブプロファイルへ戻す設計にしないと、
    「ライブ値」と「DB値」の二重管理が発生する(A4)。単一の真実の源(アクティブプロファイル)
    を経由する設計にすることで、呼び出し元の persist/refresh 漏れが実害化しなくなる。
  - WS 配信の変更検知による送信スキップ(E4)は、新規接続クライアントへの初回送信を
    別途保証しないと「値が安定した状態で接続した新規クライアントが無表示になる」
    リグレッションを生む。単純な差分検知だけでなく接続イベントとの整合を要する。
  - ログのバッチ書き込み化(E5)で SQL 側 NOW() を使うと、同一トランザクション内の
    全行が同一タイムスタンプになり時系列データとしての意味を失う。バッチ処理を
    導入する際はタイムスタンプの発生源をアプリ側に寄せる必要がある。
- **次回への改善提案**:
  - CycleLoopBase 抽出(S1)のような大規模構造変更は、既存テストの網羅性が
    そのままリファクタの安全網になった。次回も同様の抽出作業ではまず対象範囲の
    既存テストカバレッジを確認してから着手すると手戻りが少ない。
  - E1(アイドルスナップショットTTLキャッシュ)と E4(WS変更検知)は隣接する層
    (RobotController と ws.py)の最適化で、組み合わせた際の相互作用(TTL期間中は
    値が同一になりやすく E4 のスキップ判定と噛み合う)を意識すると理解しやすい。
- **ユーザーへの報告事項**（実装中に発見した仕様判断が必要な事項）:
  - D3: タイムスケジュールの同時踏みを仕様として許可するか
  - D4: コスト関数修正に伴う既存プロファイルの再チューニング推奨
  - I2: `ArchiveManager.check_and_archive()` はコード上どこからも呼び出されておらず、
    アーカイブ機能(内蔵SSD容量超過時のUSB SSDへの移行)は本番で一度も動作していない
    (潜在バグ)。ブロッキング I/O の asyncio.to_thread オフロードは実施済みだが、
    「いつ呼ぶか」(起動時／走行終了毎／専用スケジューラ等)は仕様判断が必要なため
    未配線のまま。運用上ディスク容量管理が必要であれば、配線方法を別途指示してほしい。
