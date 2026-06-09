---
name: hw-verification-init-estop-calib-design
description: 実機検証の手順設計（起動・初期化・非常停止・キャリブレーション）
metadata:
  type: project
---

# 設計: 実機検証の進め方

## 検証アプローチ

実装変更を伴わない**動作確認タスク**。本番起動経路（uvicorn + 実HWモード）で
Web API を叩き、状態遷移とアクチュエータの物理動作を確認する。

各物理動作ステップは **Claude が API を準備 → 操作者が安全確認 → 実行 →
両者で結果確認** の協調手順で行う。

## フェーズ構成

### フェーズ0: 環境レディネス確認（読み取りのみ・物理動作なし）

実機が動かせる前提を満たしているか、非破壊で点検する。

| 項目 | 確認方法 | 期待値 |
|------|---------|--------|
| アクセル/ブレーキ シリアル | `ls /dev/ttyUSB*` | ttyUSB0, ttyUSB1 存在 |
| Kvaser CAN | `lsusb` | Kvaser AB デバイス存在 |
| GPIO | `ls /dev/gpiochip*` | gpiochip0 存在 |
| PostgreSQL | `systemctl is-active postgresql` | active |
| DB スキーマ・プロファイル | `setup_db.py` 済み / プロファイル1件以上 | 接続成功 |
| settings.toml | ポート・slave_id・ピン・bitrate が実機一致 | 値が正しい |
| venv 依存 | `.venv` で import 確認 | エラーなし |

> ⚠️ 既知の懸念: Kvaser は canlib 経由のため `/dev/usbcanII*` は出ない場合が
> ある。`start()` は `can_reader.connect()` を含むため、CAN 接続失敗時は
> STANDBY に到達できず ERROR になる。フェーズ1で切り分ける。

### フェーズ1: 実HWモードでサーバ起動 → STANDBY 確認

```bash
DRIVING_ROBOT_USE_REAL_HW=1 \
DATABASE_URL=postgresql://localhost/driving_robot \
.venv/bin/uvicorn src.web.app:app --host 0.0.0.0 --port 8080
```

- lifespan で `build_real_controller()` → `controller.start()` が実行される。
- `GET /api/v1/drive/status` で `robot_state` を確認する。
  - 期待: `STANDBY`（接続・安全監視開始 成功）
  - `ERROR` の場合: 起動ログでアクチュエータ/CAN のどれが失敗したか切り分け。

### フェーズ2: 初期化検証

1. プロファイル選択（任意・キャリブレーション保存先に必要）
   `POST /api/v1/drive/select-profile {profile_id}`
2. **操作者の安全確認後** `POST /api/v1/drive/initialize`
   - 期待: 200 / `status` が READY
   - 実機: アラームリセット・サーボON（通電音/保持トルク）、
     前回非正常終了なら原点復帰動作。
3. `GET /status` で `READY` を確認。

### フェーズ3: 非常停止検証

A. **API 経由**
1. （READY から）`POST /api/v1/drive/emergency`
   - 期待: 200、`GET /status` が `EMERGENCY`、両軸が原点へ移動。
2. `POST /api/v1/drive/reset-emergency` → `READY` 復帰。

B. **GPIO 物理スイッチ経由**
1. READY 状態で物理非常停止スイッチを押す。
   - 期待: GPIO 割り込み → `controller.emergency_stop()` → `EMERGENCY` 遷移＋
     両軸原点復帰（`GET /status` で確認）。
2. スイッチ復帰後 `POST /reset-emergency` → `READY`。

> READY からの EMERGENCY 遷移は許可済み（VALID_TRANSITIONS）。
> 走行中でなくても物理スイッチで止められることを確認する。

### フェーズ4: キャリブレーション検証（手動ジョグ）

各軸（accel, brake）について:

1. `POST /api/v1/drive/calib/jog {axis, step}` で可動を確認
   （READY → 自動で CALIBRATING）。**少しずつ**動かす。
2. ゼロ点位置で `POST /calib/set-zero {axis}`
3. フル点まで jog し `POST /calib/set-full {axis}`
4. 両軸完了後 `POST /calib/save`
   - 期待: `success=true`, `data.is_valid=true`, stroke が妥当、DB 保存。
5. `GET /status` が `READY` に戻ることを確認。

代替: 自動キャリブレーション `POST /api/v1/drive/calibrate` も実機にあれば確認。

## 起動・操作の補助

検証中の API 呼び出しは `curl` または GUI（`/static/index.html`）で行う。
GUI には初期化・非常停止・キャリブレーション操作が実装済み
（`src/web/static/js/screens/`）。物理動作確認は GUI 経由が分かりやすい。

## 問題発生時の方針

- STANDBY に到達しない → 起動ログで connect 失敗箇所を特定、settings/配線/
  ドライバを確認。コード不具合なら別ステアリングで修正。
- 物理動作が想定と違う → 即時に物理非常停止、状態と差分を記録して中断。
</content>
