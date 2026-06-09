---
name: hw-verification-init-estop-calib-requirements
description: 実機（本番環境）で初期化・非常停止・キャリブレーションが動作することを確認する検証要件
metadata:
  type: project
---

# 要件: 実機検証（初期化・非常停止・キャリブレーション）

## 背景

本番コードは `.steering/20260530-production-apply` でハードウェアテスト
（`tests/hardware/`）を基準に整備済み。今回は **Web アプリ（本番起動経路）**
を実機に接続し、以下 3 操作が本番環境（Raspberry Pi 5 + 実アクチュエータ +
GPIO 非常停止 + Kvaser CAN + PostgreSQL）で実際に動作することを確認する。

1. **初期化** (`POST /api/v1/drive/initialize`)
   - アラームリセット → サーボON → （前回非正常終了時）両軸原点復帰
   - 状態遷移: STANDBY → INITIALIZING → READY
2. **非常停止** (`POST /api/v1/drive/emergency` および GPIO 物理スイッチ)
   - 即座に EMERGENCY 遷移 → 両軸原点復帰 → セッション emergency 終了
   - リセット (`POST /api/v1/drive/reset-emergency`): EMERGENCY → READY
3. **キャリブレーション**（手動ジョグ方式、`/api/v1/drive/calib/*`）
   - jog / home / set-zero / set-full / save
   - 状態遷移: READY →（自動）CALIBRATING → READY、結果が DB に保存される

## 検証の前提（本番起動経路）

`src/web/app.py` の lifespan は環境変数で実機/スタブを切り替える:

- `DRIVING_ROBOT_USE_REAL_HW=1` → `factory.build_real_controller()`（実HW）
- `DATABASE_URL` 設定時 → PostgreSQL バックエンド（プロファイル・ログ永続化）

実機検証はこの 2 つの環境変数を設定して uvicorn を起動した状態で行う。

## 安全要件（最重要）

実機検証はアクチュエータを物理的に動かす。以下を必須とする。

- **操作者の立ち会い**: 物理動作（サーボON・原点復帰・ジョグ）の各ステップは
  人が動作可否を確認してから実行する。Claude は単独でアクチュエータ動作を
  トリガーしない。
- **可動範囲の確保**: アクチュエータ可動域に人・工具・障害物がないこと。
- **非常停止スイッチを手元に**: いつでも物理非常停止を押せる体制で行う。
- **段階的実施**: 初期化 → 非常停止 → キャリブレーションの順。前段が安全に
  完了してから次へ進む。

## 受け入れ基準

- [ ] 実機環境の前提（デバイス・DB・設定）がすべて満たされている
- [ ] サーバを実HWモードで起動でき、`start()` で STANDBY まで到達する
      （アクチュエータ接続・CAN 接続・安全監視開始が成功）
- [ ] 初期化 API で READY へ遷移し、サーボON／原点復帰が実機で確認できる
- [ ] 非常停止 API・GPIO 物理スイッチの双方で EMERGENCY 遷移＋原点復帰が
      実機で確認でき、reset で READY へ復帰できる
- [ ] キャリブレーション（ジョグ→ゼロ/フル設定→保存）が実機で完了し、
      結果が is_valid=true で DB に保存される
- [ ] 検証中に観測した問題・差分を記録する

## 非スコープ

- 自動走行・学習走行・手動運転走行ループの検証
- 過電流・AC断シーケンスの検証（別ハードウェアテストで担保済み）
- UPS 監視の検証（別途）
</content>
</invoke>
