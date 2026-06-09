# タスクリスト: 非常停止の動きの修正

- [x] T1: `src/web/ws.py` の `broadcast_loop` で `get_realtime_data()` に
      `asyncio.wait_for` タイムアウトを付与し、状態配信を HW 読み取りから分離する
- [x] T2: `src/app/robot_controller.py` の `emergency_stop()` を再入安全にする
      (既に EMERGENCY なら早期 return)
- [x] T3: `tests/unit/test_robot_controller.py` に再入テストを追加
- [x] T4: `tests/unit/test_ws_broadcast.py` を新規作成し、HW 読み取りハング時も
      状態が配信されることを検証
- [x] T5: テスト・lint・型チェックを実行して全パスを確認

## 追加対応 (2026-06-09): 非常停止からの復帰ナビゲーション

報告: 非常停止画面は出るようになったが、スイッチ解除後に「初期化画面へ」を押しても変化しない。

- [x] T6: 真因特定 — `Frame` の `EmergencyOverlay`(sketch.js)「初期化画面へ」ボタンは
      `onGoInit`=`setNav('init')` のみで非常停止をリセットしないため、robotState が
      EMERGENCY のままオーバーレイ(zIndex 999)が消えない
- [x] T7: `app.js` の `onGoInit` を「reset-emergency 実行 → 初期化画面へ遷移」に変更
- [x] T8: `reset_emergency` を `EMERGENCY → STANDBY` に変更（初期化画面の「初期化を実行」=
      `initialize()` は STANDBY からのみ可能なため）。`VALID_TRANSITIONS` 更新
- [x] T9: 既存テスト更新 + 復帰シナリオ(reset→initialize→READY)テスト追加、全パス確認

## 追加対応 (2026-06-09): 物理スイッチ未解除時のリセット禁止

報告: スイッチ未解除でも「初期化画面へ」で遷移してしまう。ログでは押下・解除とも level=1。
要求: 物理スイッチが解除されている時のみ遷移。未解除なら反応しない。

真因: GPIO 割り込みを RISING_EDGE のみで登録していたため、解除(FALLING→0)を観測できず、
ログが常に level=1 だった。現在レベルを読めば 0/1 を判別できる。

- [x] T10: GPIO 監視を BOTH_EDGES に変更。`_on_emergency` を level で分岐
      (1=押下→発火 / 0=解除→ログ+フラグ更新)。解除を検知・ログ出力できるように
- [x] T11: `GPIOMonitor.is_emergency_active()` を追加（`lgpio.gpio_read` で現在レベルを
      直接読む。HIGH=押下中）。`_GpioSafetyAdapter` / `_StubSafetyMonitor` / Protocol へ伝播
- [x] T12: `reset_emergency()` でスイッチ押下中なら `EmergencyStillActive` を送出。
      ルーターは 423 Locked を返す。フロントは `if (r)` ガードで遷移しない（既存実装で対応済）
- [x] T13: テスト追加（解除エッジ非発火 / is_emergency_active live read / リセット拒否→
      解除後許可）。全 458 件パス、ruff・mypy クリーン

## 追加対応 (2026-06-09): ベンチ検証モード（非常停止スイッチのみ実機）

要望: アクチュエータ/CAN 未接続・非常停止スイッチのみの環境で実機動作を確認したい。
制約: USE_REAL_HW=1 は起動時に全 HW へ connect しに行くため、未接続だと起動が中断する。

- [x] T14: `build_real_controller(..., bench_gpio_only=False)` を追加。True で
      アクチュエータ/CAN を `_StubActuator`/`_StubCANReader` に置換、GPIO は実機のまま
- [x] T15: `app.py` lifespan で `DRIVING_ROBOT_BENCH_GPIO_ONLY=1` を読み取り配線
- [x] T16: factory テスト追加（bench 時に Actuator/CAN 未生成・GPIO 実機配線）、全 459 件パス

起動例:
```
DRIVING_ROBOT_USE_REAL_HW=1 DRIVING_ROBOT_BENCH_GPIO_ONLY=1 \
  DATABASE_URL=postgresql://localhost/driving_robot \
  .venv/bin/uvicorn src.web.app:app --host 0.0.0.0 --port 8080
```
注: UPS(NUT) は未接続でもポーリングが例外を握り潰すため起動はブロックしない（初回のみ
警告ログ）。アクチュエータ home_return 等はスタブで no-op。

## 追加対応 (2026-06-09): 全画面で非常停止を有効化

報告: 非常停止画面が初期化ページ(READY)でしか出ない。プロファイル/キャリブレーション/
走行モード/学習などでもスイッチ押下時に非常停止画面を出したい。

真因: `emergency_stop()` の `_transition(EMERGENCY)` が READY/RUNNING/MANUAL からしか許可
されておらず、CALIBRATING・PRE_CHECK・STANDBY（reset 後）等から呼ぶと InvalidStateTransition
を送出。GPIO コールバックは run_coroutine_threadsafe 経由で例外が握り潰され、状態遷移せず。
フロントのオーバーレイは全画面で robotState==='EMERGENCY' により表示されるため、真因は
バックエンド状態機械（フロント変更不要）。

- [x] T17: `VALID_TRANSITIONS` を更新し、BOOTING を除く全状態（STANDBY/INITIALIZING/
      CALIBRATING/PRE_CHECK + 既存 READY/RUNNING/MANUAL）から EMERGENCY を許可
- [x] T18: STANDBY/CALIBRATING/PRE_CHECK からの非常停止テストを追加。全 462 件パス
      （BOOTING からは引き続き例外 = 既存テスト維持）

## 申し送り事項

### 実装完了日: 2026-06-09

### 最終的な変更（全 6 群、合計 18 タスク）

1. **WS 状態配信の分離** (T1)
   - `src/web/ws.py`: `get_realtime_data()` に 0.5s タイムアウトを付与
   - HW 読み取りがストールしても `robot_state` は必ず配信

2. **`emergency_stop()` 再入防止** (T2)
   - `src/app/robot_controller.py`: 既に EMERGENCY なら早期 return
   - 多重 GPIO エッジ時の `home_return` 重複実行を防止

3. **テスト / 検証** (T3+T4)
   - 再入テスト、HW 読み取りハング時も状態配信される回帰テスト追加
   - **全 440 件パス** (当時)

4. **フロント復帰ナビゲーション** (T6+T7)
   - `app.js` の `onGoInit` を「`reset-emergency` 実行 → 遷移」に変更
   - オーバーレイが消える、初期化画面へ遷移できるように

5. **物理スイッチ状態の検知** (T8+T10-13)
   - `src/infra/gpio_monitor.py`: BOTH_EDGES 監視 + `is_emergency_active()` live read
   - `src/app/robot_controller.py`: `reset_emergency()` がスイッチ押下中なら拒否
   - スイッチ未解除で遷移しない（423 Locked）→ 解除後に遷移可能に
   - **全 458 件パス**

6. **ベンチ検証モード** (T14-16)
   - `DRIVING_ROBOT_BENCH_GPIO_ONLY=1` でアクチュエータ/CAN をスタブ化
   - スイッチ 1 本で全フロー検証可能に

7. **全状態からの非常停止対応** (T17-18)
   - `VALID_TRANSITIONS`: BOOTING 除く全状態から EMERGENCY を許可
   - プロファイル / キャリブレーション / 走行モード / 学習 など全画面で非常停止画面を表示
   - **最終 462 件パス**

### 技術的な学び

- **WebSocket 遅延分離**: 状態と計測値の配信を分断すると、HW ストール時の堅牢性が劇的に改善
- **GPIO 両エッジ監視**: RISING_EDGE のみでは解除を観測できず、level=0 の判定が不可能
- **状態機械の安全性**: 物理的安全装置は起動前を除く全状態を対象にすべき
- **ベンチモード**: 未接続環境での段階的検証が開発・テスト効率を大幅向上

### 次回への改善提案

- 実機で `level=0` ログが出ることを確認（配線が正常か切り分け可能）
- ベンチモード→アクチュエータ接続→CAN 接続と段階的に HW を繋ぐフロー確立
- さらなる堅牢化: `get_realtime_data` を独立タスク化・計測値キャッシュ化
- 非常停止時にイベント駆動で即時 WebSocket broadcast する案
