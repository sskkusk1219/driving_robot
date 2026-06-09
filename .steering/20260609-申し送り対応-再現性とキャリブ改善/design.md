---
name: handover-impl-design
description: 申し送り事項実装の設計
metadata:
  type: project
---

# 設計: 申し送り事項の実装

## A1: udev ルールのリポジトリ管理＋設置スクリプト

- `config/udev/99-driving-robot-actuators.rules.example`:
  プレースホルダーのシリアルでルールのテンプレートをコミット。
- `scripts/setup_udev.sh`:
  - `--list`: 接続中の FTDI USB-RS485-WE デバイスとシリアルを表示。
  - `<accel_serial> <brake_serial>`: 指定シリアルでルールを生成し
    `/etc/udev/rules.d/` に設置、`udevadm control --reload-rules` + `trigger`。
  - 機器固有値のためルール実体(.rules)は gitignore 済みディレクトリ扱いとし、
    .example のみコミット（settings.toml と同方針）。

## A2: setup_db マイグレーション

`calibration_data` CREATE TABLE 直後に冪等な制約追加を追加する。
Postgres は `ADD CONSTRAINT IF NOT EXISTS` 非対応のため DO ブロックを使う。

```sql
DO $$
BEGIN
    IF NOT EXISTS (
        SELECT 1 FROM pg_constraint WHERE conname = 'calibration_data_profile_id_key'
    ) THEN
        ALTER TABLE calibration_data
            ADD CONSTRAINT calibration_data_profile_id_key UNIQUE (profile_id);
    END IF;
END $$;
```

## A3: settings.toml.example

`[serial]` の accel_port/brake_port を `/dev/actuator_accel` / `/dev/actuator_brake`
に変更し、udev 設置（scripts/setup_udev.sh）を案内するコメントを付す。

## B4: jog 後の位置決め完了待ち

`ActuatorDriver.wait_for_position_complete(timeout_s)` を追加。
- DSS1(0x9005) と DSSE(0x9007) を一括読み取り（0x9005 から 3 ワード）。
- 移動指令直後は PEND が前回値で残るため、判定前に短い猶予を置く。
- `registers[0] & PEND(1<<3)` かつ `not (registers[2] & MOVE(1<<5))` で完了とみなす。
- タイムアウトで TimeoutError（home_return と同方針）。

`RobotController.ActuatorDriverProtocol` に `wait_for_position_complete()` を追加。
`jog_axis` は `move_to_position` の後に `wait_for_position_complete()` を呼んでから
`read_position` する。`DriveLoop` は `move_to_position` のみ使うため非影響。

スタブ(`_StubActuator`)・テスト mock にも no-op を追加。

## B5: 保存失敗時のリトライ導線

`save_manual_calibration` を `finally` 一括遷移から、結果分岐に変更する。

```python
if self._state != RobotState.CALIBRATING:
    raise InvalidStateTransition(...)
result = (await manager.save_manual(...)) if manager else CalibrationResult(失敗)
if not result.success:
    return result               # CALIBRATING 維持・pending 保持・原点復帰しない
await gather(accel.home_return(), brake.home_return())  # 成功時のみ
self._transition(RobotState.READY)
return result
```

- save_manual が例外（DBエラー等）を送出した場合も `finally` が無いため
  CALIBRATING を維持し、リトライ可能（pending 保持）。

## テスト

- `test_robot_controller.py`:
  - 成功時: 両軸 home_return＋READY（既存テスト維持）。
  - 失敗時: home_return 呼ばれず CALIBRATING 維持・pending 保持（テスト更新）。
  - 例外時: CALIBRATING 維持（新規）。
  - jog: wait_for_position_complete が呼ばれる（mock 追加）。
- mock(`make_accel_driver`)・スタブに wait_for_position_complete を追加。
</content>
