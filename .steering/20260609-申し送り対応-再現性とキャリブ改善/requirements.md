---
name: handover-impl-requirements
description: 実機検証の申し送り事項（再現性確保・キャリブUX改善）の実装要件
metadata:
  type: project
---

# 要件: 申し送り事項の実装

`20260609-実機検証-初期化-非常停止-キャリブレーション` の振り返りで挙げた
申し送り事項を実装する。

## スコープ

### A. 永続化変更の再現性確保
1. **A1 udev ルールのリポジトリ管理＋設置スクリプト化**
   - 手動作成した `/etc/udev/rules.d/99-driving-robot-actuators.rules` を再現可能にする。
   - 機器固有のFTDIシリアル番号を含むため、テンプレート(.example)＋設置スクリプトとする。
2. **A2 setup_db に既存テーブル向けマイグレーション追加**
   - `calibration_data` に `UNIQUE(profile_id)` が無い既存DBへ冪等に制約を追加する
     （`CREATE TABLE IF NOT EXISTS` は既存テーブルを変更しないため）。
3. **A3 settings.toml.example に /dev/actuator_* 推奨値を明記**
   - udev 固定シンボリックリンクを推奨デフォルトとして記載する。

### B. キャリブレーション操作の改善
4. **B4 jog 後の位置決め完了待ち**
   - jog 直後の `read_position` が移動途中値を返す問題を解消する。
   - 位置決め完了（DSS1 PEND）まで待ってから位置を返す。
   - 50ms 制御ループ（DriveLoop）には影響させない（jog 系のみ）。
5. **B5 保存失敗時のリトライ導線**
   - キャリブ保存失敗（バリデーション不合格・DBエラー等）時は CALIBRATING を維持し、
     記録済みゼロ/フル(pending)を保持してリトライ可能にする（原点復帰しない）。
   - 成功時のみ両軸を原点復帰して READY へ遷移する。

## 受け入れ基準

- [ ] udev ルールがリポジトリ管理され、スクリプトで再設置できる
- [ ] `setup_db.py` を既存DBに再実行すると UNIQUE 制約が冪等に追加される
- [ ] settings.toml.example が /dev/actuator_* を推奨値として示す
- [ ] jog 後に返る位置が静定値（PEND 後）になる。DriveLoop は従来どおり
- [ ] 保存失敗で CALIBRATING 維持・pending 保持・非原点復帰、成功で原点復帰＋READY
- [ ] 既存・新規ユニットテストが pass、ruff・mypy クリーン

## 非スコープ

- 自動キャリブレーション(`run_calibration`)のフロー変更（既に内部で原点復帰）
- GUI 側の変更（必要なら別タスク）
</content>
