---
name: hw-verification-init-estop-calib-tasklist
description: 実機検証タスクリスト（初期化・非常停止・キャリブレーション）
metadata:
  type: project
---

# タスクリスト: 実機検証（初期化・非常停止・キャリブレーション）

> 🔴 物理動作（サーボON・原点復帰・ジョグ）を伴うタスクは操作者の
> 安全確認後に実行する。Claude は単独でアクチュエータ動作をトリガーしない。

## フェーズ0: 環境レディネス確認（非破壊・Claude 実施可）

- [x] /dev/ttyUSB0, ttyUSB1 の存在確認 → ttyUSB0/1/2 存在
- [x] Kvaser CAN デバイス（lsusb）の存在確認 → Kvaser AB Leaf SemiPro HS 検出
- [x] /dev/gpiochip0 の存在確認 → 存在
- [x] PostgreSQL が active であることを確認 → active
- [x] DB スキーマ存在・プロファイル1件以上を確認 → 5テーブル存在・プロファイル4件
- [x] config/settings.toml の実機値（ポート・bitrate・GPIO ピン）を確認 → ttyUSB0/1, 38400, bitrate 500000, emergency_pin 17, slave_id 1
- [x] .venv で主要モジュール（pymodbus / can / lgpio / asyncpg）が import 可能か確認 → 全OK
- [x] レディネス結果を要約し、未充足項目があれば操作者へ報告 → 下記参照（懸念: Kvaser canlib ドライバ実接続はフェーズ1で確認）

## フェーズ1: 実HWモードでサーバ起動 → STANDBY 確認

- [x] DRIVING_ROBOT_USE_REAL_HW=1 + DATABASE_URL でサーバ起動 → 起動試行したが lifespan で失敗
- [x] 起動ログにエラーがないこと → ❌ ttyUSB0 ロック失敗（後述）
- [x] GET /api/v1/drive/status が STANDBY を返すことを確認 → ポート修正後に **STANDBY 到達**（CAN・両軸接続・安全監視 成功）
- [x] （ERROR の場合）失敗コンポーネントを切り分け、原因を記録 → 下記「観測した問題」参照

### 🔴 フェーズ1 で判明したブロッカー: USBシリアル割り当て衝突

- `controller.start()` の Modbus 接続が `ConnectionError: Modbus RTU 接続失敗: port=/dev/ttyUSB0`
  （`Could not exclusively lock port /dev/ttyUSB0: Resource temporarily unavailable`）で失敗。
- 原因: **`/dev/ttyUSB0` は実際には UPS シリアル（Prolific PL2303, NUT apcsmart が占有）**。
  アクチュエータ2軸は FTDI USB-RS485-WE の `/dev/ttyUSB1`(FTBB7KRI) と `/dev/ttyUSB2`(FTAQUJJJ)。
- `config/settings.toml` は accel_port=ttyUSB0 / brake_port=ttyUSB1 で実機と不一致。
- CAN(Kvaser) はteardownログから接続自体は成立していた可能性が高い。
- udev による固定割り当てルールは未設定（architecture.md が推奨していたもの）。

### 追加タスク: ポート固定割り当ての修正（要操作者の配線情報）

- [x] FTDI シリアルと軸の対応を操作者に確認 → accel=FTBB7KRI, brake=FTAQUJJJ
- [x] udev ルールで安定シンボリックリンク作成 → /etc/udev/rules.d/99-driving-robot-actuators.rules（/dev/actuator_accel, /dev/actuator_brake）
- [x] config/settings.toml の accel_port / brake_port を修正 → /dev/actuator_accel, /dev/actuator_brake
- [x] サーバ再起動 → STANDBY 到達を再確認 → 成功

## フェーズ2: 初期化検証

- [x] ~~プロファイルを選択（select-profile）~~（初期化には不要のためスキップ。キャリブ保存前にフェーズ4で選択する）
- [x] 【物理動作・要安全確認】POST /initialize を実行（サーボON／原点復帰）→ HTTP 200・Modbus 指令完走
- [x] GET /status が READY であることを確認 → READY
- [x] サーボON・原点復帰の物理動作を操作者が目視確認 → 操作者目視OK

## フェーズ3: 非常停止検証

- [x] 【物理動作・要安全確認】POST /emergency → EMERGENCY 遷移＋原点復帰を確認 → HTTP 200・EMERGENCY・原点復帰指令完走（物理目視は操作者確認）
- [x] POST /reset-emergency → READY 復帰を確認 → READY
- [x] 【物理動作・要安全確認】GPIO 物理スイッチで EMERGENCY 遷移＋原点復帰を確認 → ログ「非常停止スイッチ検知: gpio=17 level=1」・EMERGENCY・原点復帰 目視OK
- [x] スイッチ復帰後 reset-emergency → READY 復帰を確認 → スイッチ解除後 READY 復帰

## フェーズ4: キャリブレーション検証（手動ジョグ）

- [ ] 【物理動作・要安全確認】accel 軸 jog で可動確認（CALIBRATING 自動遷移）
- [ ] accel ゼロ点 set-zero / フル点 set-full を記録
- [ ] 【物理動作・要安全確認】brake 軸 jog で可動確認
- [ ] brake ゼロ点 set-zero / フル点 set-full を記録
- [x] accel/brake ジョグ・set-zero/set-full は正常動作（accel 133→2324, brake 1000→4000, 両ストローク範囲内）
- [x] POST /calib/save で success=true・is_valid=true・DB 保存を確認 → 初回 500（DBスキーマ不整合）→ 制約追加後に成功
- [x] GET /status が READY に戻ることを確認 → READY

### 🔴 フェーズ4 で判明したブロッカー2: DBスキーマのドリフト

- `POST /calib/save` が HTTP 500。バリデーションは通過(is_valid=true)し DB 書込で失敗。
- 原因: `asyncpg ... ON CONFLICT 指定に合致するユニーク制約がありません`。
  `profile_repository.save_calibration` は `INSERT ... ON CONFLICT (profile_id)` を使うが、
  本番DBの `calibration_data` に **UNIQUE(profile_id) 制約が無い**。
  `setup_db.py` は UNIQUE を定義済みだが、`CREATE TABLE IF NOT EXISTS` は既存テーブルを
  変更しないため、UNIQUE 追加前に作成された旧テーブルが残存していた（スキーマドリフト）。
- 対処: 本番DBに `ALTER TABLE calibration_data ADD CONSTRAINT calibration_data_profile_id_key
  UNIQUE (profile_id)` を実行（テーブルは0行のため安全・可逆）。

### 観測した実装上の気づき（バグではないが要検討）

- jog 直後の read_position が移動途中値を返す（指令+50→読み19 等）。静定待ち（sleep）後は
  正しい値。set-zero/set-full 前に静定待ちが無いと途中値を記録しうる。
- save 失敗時に finally で READY 遷移するが pending(zero/full) は保持される一方、
  再 CALIBRATING 突入(jog/home)で pending がリセットされる。save 失敗→再保存の
  導線がなく、ゼロ/フルの取り直しが必要になる。

### 追加タスク: 制約追加後の再保存

- [x] calibration_data に UNIQUE(profile_id) 制約を追加（0行・安全）
- [x] ゼロ/フル点を取り直し（再 CALIBRATING で pending リセットのため）→ 同一位置で再記録
- [x] POST /calib/save 再実行 → success=true・is_valid=true・DB保存を確認 → 成功
- [x] GET /status が READY を確認 → READY、DB行も確認済み

## フェーズ5: 追加機能（検証中の要望）— 保存時に両軸原点復帰

検証完了後、操作者から「キャリブレーションの保存ボタンを押したら両軸とも原点復帰して
完了するフローにしたい」との要望。`save_manual_calibration` を変更して実装する。

- [x] src/app/robot_controller.py の save_manual_calibration に保存後の両軸 home_return を追加（成否に関わらず実行）
- [x] tests/unit/test_robot_controller.py に TestSaveManualCalibration（成功時/失敗時に両軸home、READY以外で例外）を追加
- [x] make_accel_driver mock に move_to_position(AsyncMock) を追加
- [x] pytest（unit 434 passed）・ruff・mypy パス
- [x] 新コードでサーバ再起動 → STANDBY 到達
- [x] （任意）新フロー（save→両軸原点復帰）を実機で確認 → save 後に両軸 pos=0 を確認・success=true

## フェーズ6: 後片付け・記録

- [x] ~~サーバを安全に停止~~（操作者の希望で実HWモードのまま起動継続: PID 13777, port 8080, READY）
- [x] 検証結果（成功/失敗・観測値・問題）を本ファイルの振り返りに記録

## 実装後の振り返り

**検証実施日**: 2026-06-09

**結果サマリ**: 初期化・非常停止（API＋GPIO物理スイッチ）・キャリブレーション（手動ジョグ→
DB保存）の3操作すべて本番経路（実HWモード uvicorn）で動作確認完了。検証過程で本番環境の
不整合を3件発見し修正した。

**発見・修正した本番不整合**:
1. USBシリアル割り当て衝突: ttyUSB0 は UPS(Prolific/NUT apcsmart)、アクチュエータは
   FTDI ttyUSB1/2。settings.toml と不一致で起動失敗。→ udev ルール
   (99-driving-robot-actuators.rules) で /dev/actuator_accel・/dev/actuator_brake を固定、
   settings.toml をシンボリックリンク参照に修正。
2. DBスキーマドリフト: calibration_data に UNIQUE(profile_id) が無く、save の
   ON CONFLICT(profile_id) が失敗。→ 本番DBに UNIQUE 制約を追加（0行・安全）。
3. （改善要望）保存時に両軸原点復帰しない → save_manual_calibration に home_return を追加。

**計画との差分**:
- 物理動作を伴うため完全自律ではなく、各物理ステップで操作者の安全確認を挟む協調実行とした。
- フェーズ5に「保存時原点復帰」機能実装を追加（検証中の要望）。

**観測した問題・差分（要追検討）**:
- jog 直後の read_position が移動途中値を返す（静定待ちが無いと set-zero/set-full が
  途中値を記録しうる）。GUI/コントローラ側で静定待ちまたは移動完了待ちの検討余地。
- save 失敗時に pending(zero/full) は残るが、再 CALIBRATING 突入(jog/home)で pending が
  リセットされ、再保存にゼロ/フル取り直しが必要。リトライ導線の改善余地。

**次回への改善提案**:
- udev ルールと DB UNIQUE 制約をリポジトリ管理（rules ファイルを配置スクリプト化、
  setup_db に既存テーブルへの制約追加マイグレーションを用意）し、新環境で再発を防ぐ。
- settings.toml.example も /dev/actuator_* を推奨値として記載するか検討。
</content>
